/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * sdr-slc-codecd — AMBE codec service for the SDR-SLC protocol suite.
 * Example / reference code, published so you can build your own SDR-SLC device.
 *
 * Portions derived from mbelib (Pavel Yazev) and its lineage (mbelib-neo /
 * OpenDMR); AMBE frame bit-ordering follows the ETSI DMR standard.
 *
 * Copyright (C) 2026 Ivo van Ling (PA2IX)
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version. Distributed WITHOUT ANY WARRANTY. See LICENSE for the full text.
 */

/* dmr_air.h — DMR on-air AMBE+2 frame interleave.
 *
 * The SLC wire format carries the 72 AMBE bits exactly as they appear on
 * air (transmission order, MSB-first in 9 bytes) — the natural format for
 * an RF client, and the one the SDR-SLC client demonstrably sends (its
 * frames were matched byte-for-byte against an independent extraction
 * from an RF capture, and quiet-section frames sit within 11–16 bits of
 * the canonical AMBE silence pattern).
 *
 * OpenDMR's opendmr_decode()/opendmr_encode(), however, speak the DVSI
 * canonical block order: A(24, Golay 24,12) | B(23, Golay 23,12 + PRNG) |
 * C2(11) | C3(14), contiguous. On air those blocks are interleaved per
 * the standard DMR AMBE schedule (the dsd/mbelib rW/rX/rY/rZ tables).
 * Feeding on-air bits to the canonical API makes every Golay operate on
 * scrambled bits — which still "decodes" to some valid codeword, so no
 * error is ever raised and the vocoder confidently synthesises noise.
 * That was sdr-slc-codecd 0.3's decode bug, measured end-to-end: raw frames
 * through opendmr_decode gave harmonicity 0.28 (noise); the same frames
 * through dmr_air_deinterleave() first gave 0.62 (speech).
 *
 * These two functions are the entire translation, applied at the codec
 * boundary in codec.c: deinterleave on the way into the decoder,
 * interleave on the way out of the encoder (so encoded frames are
 * on-air-ready for the client's future TX path).
 */
#ifndef SLC_DMR_AIR_H
#define SLC_DMR_AIR_H
#include <stdint.h>

/* On-air order (as received off RF / on the wire) -> DVSI canonical. */
void dmr_air_deinterleave(const uint8_t in9[9], uint8_t out9[9]);

/* DVSI canonical (as produced by opendmr_encode) -> on-air order. */
void dmr_air_interleave(const uint8_t in9[9], uint8_t out9[9]);

#endif
