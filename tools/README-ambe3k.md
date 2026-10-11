# ambe3k — DVstick30 / AMBE-3000 backend (standalone, pre-integration)

`src/ambe3k.c` is a self-contained C module that talks to a DVSI **AMBE-3000**
vocoder over its serial packet interface (DVMEGA **DVstick30**, ThumbDV, NWDR
AMBE3000R/F, …). `ambe3k_loopback` is a bench harness that exercises the chip
on its own, without the daemon: use it to check a stick before blaming the
daemon, or to compare the chip with the software vocoder.

It handles exactly the boundaries the daemon speaks: DMR AMBE+2 at
**3600 bps / 2450×1150** and D-STAR AMBE at **3600 bps /
2400×1200**, both a **72-bit / 9-byte** channel frame, one 20 ms frame each
way. Add `--dstar` to any test to run it with the chip in D-STAR mode.

## Build

```bash
make ambe3k_loopback
```

Stop the daemon first (`sudo systemctl stop sdr-slc-vocoderd`): the chip's
serial port can only be used by one process at a time.

## 1. Detect the stick (no symlinks, no config)

```bash
./ambe3k_loopback --find
# -> /dev/ttyUSB0   AMBE3000F
```

Discovery scans every `/dev/ttyUSB*`, walks sysfs up to the USB device to read
its VID:PID, keeps only the bridge chips the DVstick ships with (FTDI FT-X
`0403:6015` — your unit — plus the CP2102 and older-FTDI variants), then
actively probes each with the product-ID handshake and returns the first that
answers `AMBE3000…`. VID:PID alone is never trusted; the handshake is what
confirms it. The daemon uses the same discovery, and keeps re-checking in the
background, so a stick plugged in later is picked up without a restart.

If you need a specific port, skip discovery with `--dev /dev/ttyUSB0`.

## 2. Quick self-test (1 kHz tone loopback)

```bash
./ambe3k_loopback --tone 1000 2 --out tone_out.s16
```

Synthesises a 1 kHz sine, runs PCM→AMBE→PCM through the chip, dumps the first
9-byte AMBE frame in hex, and reports the **dominant output frequency**. This
is the same check that historically caught the software encoder's bit-order
bug (1 kHz in → ~575 Hz out). If the chip's frame order matches the daemon's
on-air wire order, you'll see:

```
dominant output freq  : 1000 Hz
VERDICT: PASS - output tracks input within one bin
```

## 3. File loopback (feed your bench kit)

PCM files are raw **signed 16-bit little-endian, mono, 8000 Hz** (headerless):

```bash
# make an input (needs sox), or use your bench kit's PCM tools:
sox -n -r 8000 -c 1 -e signed -b 16 in.s16 synth 3 sine 1000

# encode + decode through the chip:
./ambe3k_loopback --in in.s16 --out out.s16

# listen:
sox -r 8000 -c 1 -e signed -b 16 out.s16 out.wav
```

## 4. Frame order (settled)

The chip's DMR channel frame is in **DMR on-air order**: it is DVSI's
"rate 33" frame, which DVSI's own test vectors carry as 9 bytes, MSB first,
DMR-interleaved (mbelib-neo's test-vector importer documents exactly this,
with the same interleave tables the daemon uses). That is why the daemon
feeds the chip the client's wire bytes verbatim, and why your on-air calls
through a DVstick30 worked with the old `--hw-deinterleave` switch off. The
switch survives as `--hw-canonical` for a chip that has been configured for
canonical order; a DVstick30 at the standard DMR RATEP does not need it.

To check a stick against off-air audio, decode a capture of on-air 9-byte
frames:

```bash
./ambe3k_loopback --decode-only --in rf_capture.ambe --out rf_decoded.s16
```

## 5. Also good to know

- **Gain:** the chip's level is the reference level. mbelib-neo decodes at
  the same level, so the daemon applies no correction to either by default.
- **FTDI latency:** the module sets `ASYNC_LOW_LATENCY` on open, dropping the
  ~16 ms FTDI latency timer that would otherwise eat the 20 ms frame budget.
- **Robustness:** every exchange flushes stale input first (one late reply
  used to shift all later exchanges by one packet), codec exchanges time out
  after 80 ms, and an unplugged stick is reported as gone rather than as a
  timeout so the daemon can drop it at once and pick it up again later.
