/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * sdr-slc-vocoderd — digital voice vocoder service for the SDR-SLC protocol suite.
 * Example / reference code, published so you can build your own SDR-SLC device.
 *
 * Copyright (C) 2026 Ivo van Ling (PA2IX)
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version. Distributed WITHOUT ANY WARRANTY. See LICENSE for the full text.
 */

/* frames.h — the wire formats, the AMBE-3000's channel frames and
 * mbelib-neo's frame representations, and the exact mappings between them.
 *
 * Three wire formats (SDR-SLC-RTP-1.0), and what each one is:
 *
 *   ambe-dmr   72 bits / 9 bytes, the AMBE+2 3600x2450 channel frame in DMR
 *              ON-AIR order (interleaved), MSB first. This is also DVSI's
 *              rate-33 channel frame, i.e. what an AMBE-3000 at the DMR RATEP
 *              consumes and produces (mbelib-neo's DVSI test-vector importer
 *              documents the vectors as "9-byte rate-33 frames, MSB first,
 *              DMR interleave", with the same tables as below). That is why
 *              the chip is fed the wire bytes verbatim by default.
 *
 *   ambe-ysf   49 bits / 7 bytes, the AMBE+2 2450 parameter bits with the
 *              AMBE FEC removed, MSB first, low 7 bits of byte 6 zero. The bit
 *              order is mbelib's ambe_d[49] order: c0 data (12) | c1 data
 *              with the PRNG removed (12) | the 25 unprotected bits.
 *
 *   ambe-dstar 72 bits / 9 bytes, the D-STAR DV voice data bytes exactly as
 *              on air (AMBE 3600x2400, LSB first within each byte), which is
 *              also the AMBE-3000's D-STAR channel frame.
 *
 * mbelib-neo works on two representations: the 72-bit frame as 4x24 bit
 * planes `char fr[4][24]` (row 0 = C0 with col 23 the MSB, row 1 = C1 col 22,
 * row 2 = C2 col 10, row 3 = C3 col 13), and the 49 parameter bits
 * `char d[49]`. Every function here is a pure bit permutation (or a call to
 * mbelib-neo's own FEC encoder), so nothing is lost in any direction. */
#ifndef SLC_FRAMES_H
#define SLC_FRAMES_H
#include <stdint.h>

#define FRAMES_AMBE72_BYTES 9
#define FRAMES_VOICE49_BYTES 7

/* DMR on-air 9 bytes <-> mbelib-neo 4x24 planes. */
void frames_dmr_air_to_fr(const uint8_t air9[9], char fr[4][24]);
void frames_fr_to_dmr_air(const char fr[4][24], uint8_t air9[9]);

/* DMR on-air <-> DVSI canonical block order (A|B|C2|C3 contiguous). Only
 * needed for an AMBE-3000 that has been configured for canonical frames
 * (--hw-canonical); the DVstick30 at the standard DMR RATEP speaks on-air. */
void frames_dmr_air_to_canonical(const uint8_t air9[9], uint8_t can9[9]);
void frames_dmr_canonical_to_air(const uint8_t can9[9], uint8_t air9[9]);

/* YSF 7 bytes <-> 49 parameter bits. */
void frames_voice49_to_d(const uint8_t v7[7], char d[49]);
void frames_d_to_voice49(const char d[49], uint8_t v7[7]);

/* 49 parameter bits -> AMBE+2 72-bit on-air frame (adds Golay + C1 PRNG),
 * and back with FEC decoding. *errs (optional) receives the corrected count.
 * Return 0 on success, -1 on an mbelib-neo argument error. */
int frames_d_to_dmr_air(const char d[49], uint8_t air9[9]);
int frames_dmr_air_to_d(const uint8_t air9[9], char d[49], int *errs);

/* D-STAR DV data bytes <-> 4x24 planes (thin wrappers over mbelib-neo). */
int frames_dstar_to_fr(const uint8_t dv9[9], char fr[4][24]);
int frames_fr_to_dstar(const char fr[4][24], uint8_t dv9[9]);

#endif
