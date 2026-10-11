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

/* dmr_prio.h — corrective transform for stock OpenDMR's encoder output, and
 * the 49-bit AMBE+2 voice-frame boundary built from the same primitives.
 *
 * Stock OpenDMR's opendmr_encode() packs the b[0..8] parameter vector
 * CONTIGUOUSLY into the 49 AMBE+2 channel bits. The real channel-bit order
 * is mbelib's priority order (each parameter's high bits first, all low
 * bits collected at the end) — the order its own decoder, and every real
 * AMBE decoder, gathers from. Stock frames therefore carry valid Golay FEC
 * around systematically scrambled parameters: measured 2026-08-23 by tone
 * loopback (1 kHz in -> encode -> decode: dominant 575 Hz, tone SNR
 * -25.6 dB) and as loud wrong-pitch garble on a real MD-380.
 *
 * Rather than patching OpenDMR, codecd corrects the frame after the fact.
 * The transform is lossless: the stock frame's FEC is valid by
 * construction and both Golay codes are systematic, so the 49 bits are
 * recovered exactly (no error correction involved), reordered, and
 * re-protected. Reported upstream; when OpenDMR fixes its packing, this
 * becomes a measurable no-op and can be retired (see codec.c).
 *
 * The 49-bit voice boundary
 * --------------------------------------
 * YSF V/D mode 2 does not use AMBE's built-in FEC on air; it re-protects the
 * raw 49 voice bits with its own VCH channel coding, done in the client. So
 * a YSF stream needs the 49-bit voice frame, not the 72-bit coded frame.
 *
 * The three functions below expose that boundary. They reuse the exact
 * primitives the "fix" already contains — a systematic Golay(24,12) and
 * mbelib's B-block PRNG — so nothing new touches the FEC. The 49 bits are
 * defined as the mbelib PRIORITY-order voice vector d[0..48], packed
 * MSB-first into 7 bytes with the low 7 bits of the last byte zero (§7.1):
 *
 *     d[0..11]  = c0 (Golay(24,12) data of block A)
 *     d[12..23] = c1 (Golay(23,12) data of block B, PRNG removed)
 *     d[24..48] = C  (25 unprotected channel bits)
 *
 * dmr_prio_fix_stock_frame() is now the composition
 * dmr_prio_stock_to_voice49() followed by dmr_prio_voice49_to_canonical(),
 * so its output is unchanged (regression-checked byte-for-byte).
 */
#ifndef SLC_DMR_PRIO_H
#define SLC_DMR_PRIO_H
#include <stdint.h>

/* In: 72-bit canonical frame as produced by stock opendmr_encode().
 * Out: 72-bit canonical frame with the channel bits in true priority
 * order. In-place (in9 == out9) is allowed. */
void dmr_prio_fix_stock_frame(const uint8_t in9[9], uint8_t out9[9]);

/* Stock (buggy-packed) canonical frame -> 49-bit priority voice frame.
 * Undoes stock OpenDMR's contiguous packing to recover the true priority
 * order, then emits it as 7 bytes. Use on the software encode path, where
 * the input is opendmr_encode()'s raw output. */
void dmr_prio_stock_to_voice49(const uint8_t stock9[9], uint8_t voice7[7]);

/* Correct (already-priority) canonical frame -> 49-bit priority voice frame.
 * Reads c0|c1|C straight out via de-Golay/de-PRNG, no parameter reorder.
 * Use on the AMBE-3000 encode path, where the chip's frame is already in
 * true order, and on any frame that was decoded off air. */
void dmr_prio_canonical_to_voice49(const uint8_t canonical9[9], uint8_t voice7[7]);

/* 49-bit priority voice frame -> correct canonical 72-bit frame.
 * Re-protects with Golay(24,12)/Golay(23,12)+PRNG. Exact inverse of
 * dmr_prio_canonical_to_voice49(). Use on both decode paths (feed the
 * result to opendmr_decode(), or interleave to on-air for the chip). */
void dmr_prio_voice49_to_canonical(const uint8_t voice7[7], uint8_t canonical9[9]);

#endif
