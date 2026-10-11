# Reference code from sdr-slc-codecd

`dmr_prio.c/.h` and `codecd_dmr_air.c` / `dmr_air.h` are copied unchanged from
[sdr-slc-codecd] at commit `e9fb528`. They are not part of the daemon. They are
the reference that `tests/test_frames.c` checks the new mappings in
`src/frames.c` against: codecd's on-air interleave was validated against a real
MD-380 capture, and its 49/72-bit conversion carried YSF on air. Agreement bit
for bit means the new daemon puts exactly the same bits on the wire.

[sdr-slc-codecd]: https://github.com/pa2ix/sdr-slc-codecd
