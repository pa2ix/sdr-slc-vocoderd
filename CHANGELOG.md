# sdr-slc-vocoderd 1.0.1

- `ping` is answered (`ok`, empty result) per SDR-SLC-CP. 1.0.0, like
  codecd, replied `unknown_command`. Pings are logged at `-vv` only.

# sdr-slc-vocoderd 1.0.0

First release. Successor of sdr-slc-codecd 1.0, forked from its commit
`e9fb528`; the SDR-SLC control plane and RTP behaviour are unchanged.

- Software vocoder is now mbelib-neo 2.4.0 (pinned as a submodule) instead of
  OpenDMR: decode and encode for DMR, YSF and D-STAR. D-STAR no longer needs
  an AMBE-3000. Lost frames are concealed per TIA-102.BABA-1.
- The AMBE-3000 is shared on voice activity: transmit claims it only while
  there is audio, receive claims it on a run of clean voice frames at any
  point in an over, with a silent warm-up and a one-frame crossfade.
- AMBE-3000 driver: stale input flushed before every exchange (fixes the
  late-reply cascade that kept the chip unused until restart), 80 ms codec
  timeout, acknowledgement status checked, device loss detected.
- The chip is re-detected in the background: plugged in later, or after it
  disappears or stops answering.
- 16 streams (was 4), TCP keepalive on control connections, and a `busy` error
  that lists the slot holders.
- Clean shutdown on SIGTERM regardless of which thread receives it.
- New: `ambe3k_sim` (simulated chip on a pty), unit tests, an end-to-end
  test, and a ThreadSanitizer target.
- Removed with OpenDMR: `--encode-gain-adjust`, the -17.5 dB decode default,
  and the `dmr_prio` re-packing. `--hw-deinterleave` is now `--hw-canonical`
  (the old name still works).
