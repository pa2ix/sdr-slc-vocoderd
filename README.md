# sdr-slc-vocoderd

The digital-voice vocoder service of the SDR-SLC suite. It turns DMR, YSF and
D-STAR voice frames into 8 kHz audio and back, for any SDR-SLC client on the
network, over the SDR-SLC control plane and RTP.

It uses a **DVSI AMBE-3000** (DVMEGA DVstick30, ThumbDV and compatibles)
whenever one is plugged in, and **[mbelib-neo]** for every stream the chip is
not serving, frame by frame and without gaps. If no chip is present, everything
runs on mbelib-neo; plug one in later and it is picked up automatically.

It is the successor of [sdr-slc-codecd], forked from its commit `e9fb528`. The
protocol side is unchanged, so clients that work with codecd work with this
daemon without changes.

| | |
|---|---|
| Formats | `ambe-dmr` (AMBE+2, 72-bit on-air frame), `ambe-ysf` (AMBE+2, 49-bit V/D mode 2), `ambe-dstar` (AMBE, 72-bit DV frame), each to and from `pcm8k` |
| Hardware | AMBE-3000 on USB serial (FTDI or CP2102), found automatically |
| Software | mbelib-neo 2.4 (GPL-2.0-or-later), decode **and** encode for all three formats |
| Streams | 16 concurrent conversions |
| Discovery | mDNS `_slc-codec._tcp`, TCP port 4630 |
| License | GPL-2.0-or-later |

## What changed compared to sdr-slc-codecd

**The chip goes to whoever is actually carrying voice.** codecd gave the chip
to transmit whenever a transmit stream existed, and let a receive stream take
it only at the very first frame of a transmission. A client that keeps its
transmit conversion "armed" with silence between overs, and sends receive
frames continuously, therefore decoded almost everything in software. One log
from the SDR-SLC client showed the armed transmit on the chip for 18,844 frames
of silence while the receiver decoded 8,696 of 8,699 frames in software. With
the same traffic, this daemon puts the receiver on the chip (291 of 300 frames
in the test that reproduces it) and the silent transmit stays in software.

**A late chip reply no longer breaks the chip until restart.** codecd's driver
did not discard a reply that arrived after its timeout, so every later
exchange read the previous exchange's reply. Decodes still looked successful
but returned the previous frame, and every later attempt to hand the chip to a
new stream failed, so all new transmissions were decoded in software until the
daemon restarted. Reproduced against the simulated chip with codecd's original
driver: after one 300 ms late reply, 5 of 5 ownership changes failed. The new
driver flushes before every exchange: 5 of 5 succeed.

**mbelib-neo replaces OpenDMR.** It decodes and encodes all three formats
(D-STAR included, so D-STAR now works without a chip), handles lost frames per
TIA-102.BABA-1 (repeat up to three times, then mute), and produces the same
audio level as the AMBE-3000, so the `--decode-gain-db -17.5`,
`--encode-gain-adjust` and bit-reordering workarounds codecd needed for OpenDMR
are gone.

**No more `busy`.** 16 streams instead of 4, dead control connections are
detected with TCP keepalive (about 25 s), and when all slots really are taken
the `busy` error lists who holds them:

```
busy: all 16 codec streams are in use: cv-23ce54fc pcm8k->ambe-dmr from 192.168.1.164 (last frame 1s ago, open 15s); ...
```

**The chip survives being unplugged.** Every stream continues in software at
once, and the chip is taken back into use when it reappears.

## How the chip is shared

The AMBE-3000 has one vocoder context, so it serves one stream at a time. All
other streams run on their own mbelib-neo instance.

- **Transmit** takes the chip when its audio rises above `--tx-vad-dbfs`
  (default -50 dBFS) and keeps it until it has been quiet for `--tx-hang-ms`
  (default 2 s). Transmit always beats receive.
- **Receive** takes the chip when its frames show a real transmission: six
  consecutive clean voice frames (120 ms) with at most 6 corrected bit errors
  in total. Random frames (an idle timeslot, an open squelch) pass this about
  once every three hours; a real signal at 2% bit errors passes 93% of the
  time. It can take the chip at any point in an over, not only at the start,
  and keeps it until `--rx-hang-ms` (default 300 ms) after the last good
  frame. The first receiver keeps the chip for its whole over.
- **No click on takeover.** A receiver that takes the chip mid-over first runs
  it silently for 3 frames (`--warmup-frames`) while the software decoder keeps
  playing, then crossfades from software to chip over one 20 ms frame. Both
  produce the same level.

Each change is logged, so you can always see which vocoder is in use:

```
[codec] cv-09be62ca RX(decode) -> AMBE-3000 chip
[codec] cv-23ce54fc TX(encode) -> AMBE-3000 chip
[codec] cv-09be62ca RX(decode) -> mbelib-neo 2.4.0
```

## Building (Debian, Ubuntu, Raspberry Pi OS)

```bash
sudo apt install build-essential cmake pkg-config libjson-c-dev libavahi-client-dev
git clone --recursive https://github.com/pa2ix/sdr-slc-vocoderd
cd sdr-slc-vocoderd
make mbelib          # builds the pinned mbelib-neo (v2.4.0) into ./build
make
make test            # unit tests, about 30 s
sudo make install    # /usr/local/sbin + systemd unit
sudo systemctl enable --now sdr-slc-vocoderd
```

If mbelib-neo is already installed system-wide (pkg-config `libmbe-neo`,
version 2.4 or later), skip `make mbelib`. mbelib-neo is linked statically
when its static library is available.

The AMBE-3000 is a serial device: the daemon's user needs access to
`/dev/ttyUSB*`, which on Debian means the `dialout` group. The systemd unit
adds it. If access is denied, the daemon says so and runs on mbelib-neo.

## Options

```
  --port N               SLC-CP TCP port (default 4630)
  --name NAME            device name in mDNS and hello (default SLC-VOCODER-<host>)

AMBE-3000:
  --no-hw                never use an AMBE-3000
  --hw-dev PATH          use PATH instead of scanning USB
  --hw-canonical         chip uses DVSI canonical DMR bit order (not needed for
                         a DVstick30; alias --hw-deinterleave)
  --hw-decode-gain-db N  chip decode output gain (default 0)
  --hw-encode-gain-db N  chip encode input gain (default 0)

mbelib-neo:
  --decode-gain-db DB    gain on software-decoded audio (default 0)
  --encode-gain-db DB    gain on audio into the software encoder (default 0)

Chip sharing:
  --tx-vad-dbfs DB       transmit claims the chip above this level (default -50)
  --tx-hang-ms MS        transmit keeps it this long after voice (default 2000)
  --rx-hang-ms MS        receive keeps it this long after good frames (default 300)
  --warmup-frames N      silent chip frames before a receive crossfade (default 3)

  -v / -vv               control plane and chip ownership / every frame
```

To compare with codecd on the same machine, run one of the two at a time, or
give this one `--port 4631`.

## Testing without a DVstick

`ambe3k_sim` is a simulated AMBE-3000 on a pseudo-terminal. It speaks the real
serial protocol (with mbelib-neo standing in for the silicon), so the complete
daemon, driver included, can be exercised on any machine:

```bash
make ambe3k_sim
./ambe3k_sim /tmp/ambe &
./sdr-slc-vocoderd --hw-dev /tmp/ambe -v
```

Type `late`, `drop`, `unplug` or `replug` into the simulator to inject faults.

| Test | What it proves |
|---|---|
| `make test` | frame mappings bit-identical to codecd's on-air-proven code (20,000 random frames); tone loopback at the right frequency and level in all formats; noise never claims the chip; erasures repeat then mute; the arbiter in every sharing scenario, with a stress run that checks the chip's state is never shared between streams |
| `make e2e` | the real daemon over SLC-CP and RTP against `ambe3k_sim`: armed transmit, preemption, a late reply, unplug and replug, D-STAR and YSF, `busy`, dropped connections, clean shutdown |
| `make tsan` | the arbiter test under ThreadSanitizer |

`ambe3k_loopback` tests a real stick on its own; see
[tools/README-ambe3k.md](tools/README-ambe3k.md).

## Wire formats

| Format | Bits / bytes | Content |
|---|---|---|
| `ambe-dmr` | 72 / 9 | AMBE+2 3600x2450 channel frame in DMR on-air (interleaved) order, MSB first. This is also the AMBE-3000's own DMR frame, so it goes to the chip unchanged. |
| `ambe-ysf` | 49 / 7 | AMBE+2 parameter bits without AMBE FEC (YSF V/D mode 2), MSB first, last 7 bits zero |
| `ambe-dstar` | 72 / 9 | D-STAR DV voice data bytes as on air (LSB first), also the AMBE-3000's D-STAR frame |
| `pcm8k` | 2560 / 320 | 160 samples, signed 16-bit big-endian, 8 kHz |

An RTP packet with an empty payload is an erasure: the client knows the frame
is bad, and the daemon conceals it instead of decoding garbage.

## Licensing and patents

GPL-2.0-or-later, the same as mbelib-neo. As mbelib-neo's own notice says, the
AMBE vocoder algorithms may be covered by patents in your country; check before
using or distributing binaries. This was equally true of OpenDMR in codecd.

## Credits

- [mbelib-neo] by arancormonk, the software vocoder, descended from mbelib.
- The AMBE-3000 packet framing was cross-checked against f4exb's serialDV.
- The DMR interleave tables come from the dsd/mbelib lineage and were validated
  against a real MD-380 capture in sdr-slc-codecd.

[mbelib-neo]: https://github.com/arancormonk/mbelib-neo
[sdr-slc-codecd]: https://github.com/pa2ix/sdr-slc-codecd
