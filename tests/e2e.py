#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# sdr-slc-vocoderd — Copyright (C) 2026 Ivo van Ling (PA2IX)
#
# End-to-end test: the real daemon and AMBE-3000 driver against ambe3k_sim
# (a simulated chip on a pty), driven over SLC-CP and RTP like the client.
#
#   1  capabilities: all formats, 16 streams, chip reported
#   2  armed TX (silence) + RX over: RX decodes on the chip, TX stays software
#   3  TX speech preempts; RX continues in software; RX returns after unkey
#   4  a late chip reply (the codecd cascade bug): one frame in software,
#      then back on the chip; every later chip exchange still lines up
#   5  unplug mid-over: no frames lost; replug: chip used again
#   6  D-STAR and YSF decode on the chip
#   7  busy: the 17th stream is refused with a list of holders
#   8  a dropped control connection releases its streams
#   9  SIGTERM: clean shutdown
import json, os, signal, socket, struct, subprocess, sys, tempfile, threading, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 46300 + os.getpid() % 500
fails = 0

def check(cond, msg):
    global fails
    if not cond:
        fails += 1
        print("   FAIL:", msg)
    return cond

def frames(fmt, kind, n):
    out = subprocess.run([os.path.join(ROOT, "tests/gen_frames"), fmt, kind, str(n)], capture_output=True, check=True).stdout
    size = 320 if fmt == "pcm" else 7 if fmt == "ysf" else 9
    return [out[i:i + size] for i in range(0, len(out), size)]

class Log:
    """The daemon's stderr, collected in the background."""
    def __init__(self, f):
        self.lines, self.lock = [], threading.Lock()
        threading.Thread(target=self._pump, args=(f,), daemon=True).start()
    def _pump(self, f):
        for line in iter(f.readline, b""):
            with self.lock:
                self.lines.append(line.decode(errors="replace").rstrip())
    def mark(self):
        with self.lock:
            return len(self.lines)
    def since(self, m):
        with self.lock:
            return self.lines[m:]
    def wait_for(self, text, m=0, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            if any(text in l for l in self.since(m)):
                return True
            time.sleep(0.05)
        return False

class Client:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", PORT))
        self.n = 0
    def cmd(self, command, **params):
        self.n += 1
        req = {"type": "command", "id": str(self.n), "command": command, "params": params}
        data = json.dumps(req).encode()
        self.s.sendall(struct.pack(">I", len(data)) + data)
        while True:
            hdr = self._recv(4)
            msg = json.loads(self._recv(struct.unpack(">I", hdr)[0]))
            if msg.get("type") == "response":
                return msg
    def _recv(self, n):
        b = b""
        while len(b) < n:
            c = self.s.recv(n - len(b))
            if not c:
                raise ConnectionError("closed")
            b += c
        return b
    def close(self):
        self.s.close()

class Stream:
    def __init__(self, cl, frm, to):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", 0))
        self.rx.settimeout(0.002)
        r = cl.cmd("open_stream", function="codec", protocol="rtp", **{"from": frm, "to": to},
                   rtp={"output": {"destination": {"ip": "127.0.0.1", "port": self.rx.getsockname()[1]}}})
        self.resp = r
        if r["status"] != "ok":
            self.id = None
            return
        res = r["result"]
        self.id = res["stream_id"]
        self.dst = ("127.0.0.1", res["rtp"]["input"]["receive"]["port"])
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq, self.ts, self.got = 0, 0, []
        cl.cmd("start_codec", stream_id=self.id)
    def send(self, payload):
        hdr = struct.pack(">BBHII", 0x80, 97, self.seq & 0xFFFF, self.ts & 0xFFFFFFFF, 0x1234)
        self.tx.sendto(hdr + payload, self.dst)
        self.seq += 1
        self.ts += 160
    def drain(self):
        while True:
            try:
                d = self.rx.recv(2048)
                self.got.append(d[12:])
            except (socket.timeout, BlockingIOError):
                return

def run(pairs, n):
    """Send n frames in real time (20 ms) on several streams at once.
    pairs: list of (stream, list_of_payloads_or_callable)."""
    t0 = time.time()
    for k in range(n):
        for st, src in pairs:
            st.send(src[k % len(src)] if not callable(src) else src(k))
        for st, _ in pairs:
            st.drain()
        sleep = t0 + (k + 1) * 0.02 - time.time()
        if sleep > 0:
            time.sleep(sleep)
    time.sleep(0.1)
    for st, _ in pairs:
        st.drain()

def backend_lines(log, m, sid):
    return [l for l in log.since(m) if l.startswith("[codec] " + sid)]

def main():
    tmp = tempfile.mkdtemp(prefix="vocoderd-e2e-")
    link = os.path.join(tmp, "ambe")
    sim = subprocess.Popen([os.path.join(ROOT, "ambe3k_sim"), link], stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    for _ in range(100):
        if os.path.islink(link):
            break
        time.sleep(0.02)
    def sim_cmd(c):
        sim.stdin.write((c + "\n").encode()); sim.stdin.flush()
    def sim_stats():
        sim_cmd("stats")
        line = sim.stdout.readline().decode()
        return dict(kv.split("=") for kv in line.split()[1:])

    d = subprocess.Popen([os.path.join(ROOT, "sdr-slc-vocoderd"), "--port", str(PORT), "--hw-dev", link,
                          "--tx-hang-ms", "400", "--name", "E2E", "-v"], stderr=subprocess.PIPE)
    log = Log(d.stderr)
    try:
        check(log.wait_for("AMBE-3000 available"), "daemon did not find the simulated chip")
        check(log.wait_for("SLC-CP on TCP"), "daemon did not start listening")
        cl = Client()
        cl.cmd("hello", supported_protocol_versions=["1.0"])

        print("1  capabilities")
        caps = cl.cmd("get_capabilities")["result"]["capabilities"]["codec"]
        ids = [f["id"] for f in caps["formats"]]
        print("   formats", ids, "capacity", caps["capacity"], "backend", caps["implementation"]["backend_version"])
        check(ids == ["pcm8k", "ambe-dmr", "ambe-ysf", "ambe-dstar"], "formats")
        check(caps["capacity"] == 16, "capacity")
        check(caps["implementation"]["backend"] == "AMBE3000", "backend should report the chip")

        voice = frames("dmr", "voice", 500)
        silence = [bytes(320)]
        speech_pcm = frames("pcm", "voice", 200)

        print("2  armed TX (silence) + RX with a real over")
        tx = Stream(cl, "pcm8k", "ambe-dmr")
        rx = Stream(cl, "ambe-dmr", "pcm8k")
        m = log.mark()
        run([(tx, silence), (rx, voice)], 150)
        rxl, txl = backend_lines(log, m, rx.id), backend_lines(log, m, tx.id)
        print("  ", "; ".join(rxl + txl))
        check(any("AMBE-3000 chip" in l for l in rxl), "RX never got the chip")
        check(not any("AMBE-3000 chip" in l for l in txl), "silent armed TX took the chip")
        check(len(rx.got) == 150 and all(len(p) == 320 for p in rx.got), "RX returned %d PCM frames of 150" % len(rx.got))
        check(len(tx.got) == 150 and all(len(p) == 9 for p in tx.got), "TX returned %d AMBE frames of 150" % len(tx.got))
        lvl = max(max(abs(v) for v in struct.unpack(">160h", p)) for p in rx.got[-50:])
        check(lvl > 1000, "decoded audio is silent (peak %d)" % lvl)

        print("3  TX speech preempts; RX continues in software; RX back after unkey")
        m = log.mark()
        rx.got.clear()
        run([(tx, speech_pcm), (rx, voice)], 100)
        rxl, txl = backend_lines(log, m, rx.id), backend_lines(log, m, tx.id)
        print("  ", "; ".join(txl + rxl))
        check(any("AMBE-3000 chip" in l for l in txl), "TX speech did not take the chip")
        check(any("mbelib-neo" in l for l in rxl), "RX did not move to software")
        check(len(rx.got) == 100, "RX lost frames during preemption (%d/100)" % len(rx.got))
        m = log.mark()
        run([(tx, silence), (rx, voice)], 60)       # unkey; tx hang is 400 ms
        rxl = backend_lines(log, m, rx.id)
        print("  ", "; ".join(rxl))
        check(any("AMBE-3000 chip" in l for l in rxl), "RX did not get the chip back after unkey")

        print("4  a late chip reply (the stale-reply cascade)")
        s0 = sim_stats()
        m = log.mark()
        sim_cmd("late")
        rx.got.clear()
        run([(tx, silence), (rx, voice)], 100)
        s1 = sim_stats()
        rxl = backend_lines(log, m, rx.id)
        dec = int(s1["decode"]) - int(s0["decode"])
        print("   chip decoded %d of 100 frames after a 150 ms late reply; %s" % (dec, "; ".join(rxl) or "no backend change"))
        check(len(rx.got) == 100, "frames lost around the late reply (%d/100)" % len(rx.got))
        check(dec >= 90, "chip exchanges did not recover after the late reply (%d)" % dec)
        check(not any("stopped answering" in l for l in log.since(m)), "a single late reply took the chip down")

        print("5  unplug mid-over, replug")
        m = log.mark()
        sim_cmd("unplug")
        rx.got.clear()
        run([(tx, silence), (rx, voice)], 75)
        check(log.wait_for("disappeared", m, 2), "daemon did not notice the unplug")
        check(len(rx.got) == 75, "frames lost across the unplug (%d/75)" % len(rx.got))
        m = log.mark()
        sim_cmd("replug")
        check(log.wait_for("AMBE-3000 available", m, 8), "daemon did not pick the chip up again")
        m = log.mark()
        run([(tx, silence), (rx, voice)], 50)
        rxl = backend_lines(log, m, rx.id)
        print("   unplug: 75/75 frames delivered; replug: %s" % ("; ".join(rxl) or "?"))
        check(any("AMBE-3000 chip" in l for l in rxl), "RX not back on the chip after replug")

        print("6  D-STAR and YSF on the chip")
        for fmt, name in (("dstar", "ambe-dstar"), ("ysf", "ambe-ysf")):
            st = Stream(cl, name, "pcm8k")
            m = log.mark()
            run([(tx, silence), (st, frames(fmt, "voice", 60))], 60)
            l = backend_lines(log, m, st.id)
            print("   %s: %s, %d PCM frames" % (name, "; ".join(l), len(st.got)))
            check(any("AMBE-3000 chip" in x for x in l), name + " did not get the chip")
            check(len(st.got) == 60, name + " lost frames")
            cl.cmd("close_stream", stream_id=st.id)

        print("7  busy")
        extra = []
        while True:
            st = Stream(cl, "ambe-dmr", "pcm8k")
            if st.id is None:
                break
            extra.append(st)
        err = st.resp.get("error", {})
        print("   %d streams open, then: %s: %s..." % (2 + len(extra), err.get("code"), err.get("message", "")[:120]))
        check(2 + len(extra) == 16, "expected 16 streams, got %d" % (2 + len(extra)))
        check(err.get("code") == "busy" and "from 127.0.0.1" in err.get("message", ""), "busy error lacks the holder list")
        for st in extra:
            cl.cmd("close_stream", stream_id=st.id)

        print("8  a dropped control connection releases its streams")
        c2 = Client(); c2.cmd("hello", supported_protocol_versions=["1.0"])
        for _ in range(3):
            Stream(c2, "ambe-dmr", "pcm8k")
        m = log.mark()
        c2.s.shutdown(socket.SHUT_RDWR); c2.close()
        check(log.wait_for("released 3 streams", m, 3), "streams of the dropped connection were not released")

        print("9  SIGTERM")
        cl.close()
        d.send_signal(signal.SIGTERM)
        try:
            rc = d.wait(5)
            print("   exited with", rc)
            check(rc == 0, "non-zero exit")
        except subprocess.TimeoutExpired:
            check(False, "daemon did not exit on SIGTERM")
    finally:
        if d.poll() is None:
            d.kill()
        sim_cmd("quit")
        sim.wait(2)
    if fails:
        print("e2e: %d failure(s); daemon log follows" % fails)
        print("\n".join(log.lines[-60:]))
        return 1
    print("e2e OK")
    return 0

if __name__ == "__main__":
    sys.exit(main())
