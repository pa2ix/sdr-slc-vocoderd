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

/* dmr_prio.c — see dmr_prio.h. Tables generated from mbelib's
 * mbe_processAmbe2450Data() gather and identity-verified:
 * gather(fix(stock_encode(b))) == b over 3000 random parameter vectors.
 *
 * The fix is split into three reusable steps so the 49-bit voice boundary
 * (dmr_prio_*_voice49) shares the exact same Golay/PRNG primitives:
 * dmr_prio_fix_stock_frame() is stock_to_priority() followed by
 * priority_to_canonical(). */
#include "dmr_prio.h"
#include <string.h>

static const int LENS[9] = {7, 5, 5, 9, 7, 5, 4, 4, 3};

/* For each of the 49 priority-order channel bits: which parameter, and
 * which bit of it (shift from LSB). The inverse of mbelib's gather. */
static const uint8_t CHAN_PARAM[49] = {
    0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8,
    1, 2, 0, 0, 0, 3, 4, 4, 4, 5, 6, 7, 8, 8
};
static const uint8_t CHAN_SHIFT[49] = {
    6, 5, 4, 3, 4, 3, 2, 1, 4, 3, 2, 1, 8, 7, 6, 5, 4, 3, 2, 1,
    6, 5, 4, 3, 4, 3, 2, 1, 3, 2, 1, 3, 2, 1, 2,
    0, 0, 2, 1, 0, 0, 2, 1, 0, 0, 0, 0, 1, 0
};

/* Golay(24,12), generator 0xC75, systematic, overall parity in bit 0 —
 * matches OpenDMR's CGolay24128::encode24128. (23,12) is the same without
 * the parity bit. */
static uint32_t golay24_encode(uint32_t data12)
{
    uint32_t v = data12 << 11;
    for (int i = 11; i >= 0; i--)
        if (v & (1u << (i + 11)))
            v ^= 0xC75u << i;
    uint32_t cw = (data12 << 12) | ((v & 0x7FF) << 1);
    return cw | (__builtin_popcount(cw) & 1u);
}

/* mbelib's B-block PRNG, seeded from the 12-bit C0 data. */
static uint32_t prng_mask_23(uint32_t c0)
{
    uint16_t pr = (uint16_t)(16u * c0);
    uint32_t mask = 0;
    for (int i = 1; i <= 23; i++) {
        pr = (uint16_t)((173u * pr + 13849u) % 65536u);
        if (pr >> 15)
            mask |= 1u << (23 - i);
    }
    return mask;
}

/* ---- The three reusable steps ------------------------------------------ */

/* Stock (buggy contiguous) canonical frame -> 49 priority-order bits.
 * Opens A|B|C, removes the PRNG, undoes stock's contiguous parameter
 * packing, then re-gathers into true priority order. */
static void stock_to_priority(const uint8_t in9[9], uint8_t d[49])
{
    uint32_t A = ((uint32_t)in9[0] << 16) | ((uint32_t)in9[1] << 8) | in9[2];
    uint32_t B = (((uint32_t)in9[3] << 16) | ((uint32_t)in9[4] << 8) | in9[5]) >> 1;
    uint32_t C = (((uint32_t)in9[5] & 1u) << 24) | ((uint32_t)in9[6] << 16)
               | ((uint32_t)in9[7] << 8) | in9[8];
    uint32_t c0 = A >> 12;
    uint32_t c1 = (B ^ prng_mask_23(c0)) >> 11;

    /* The stock encoder packed b[0..8] contiguously into c0|c1|C: unpack. */
    uint8_t contig[49];
    for (int i = 0; i < 12; i++) contig[i]      = (c0 >> (11 - i)) & 1u;
    for (int i = 0; i < 12; i++) contig[12 + i] = (c1 >> (11 - i)) & 1u;
    for (int i = 0; i < 25; i++) contig[24 + i] = (C  >> (24 - i)) & 1u;
    int b[9];
    for (int p = 0, pos = 0; p < 9; p++) {
        int v = 0;
        for (int j = 0; j < LENS[p]; j++)
            v = (v << 1) | contig[pos++];
        b[p] = v;
    }

    /* Re-emit in true priority order. */
    for (int i = 0; i < 49; i++)
        d[i] = (uint8_t)((b[CHAN_PARAM[i]] >> CHAN_SHIFT[i]) & 1u);
}

/* Correct (already-priority) canonical frame -> 49 priority-order bits.
 * c0|c1|C ARE d[] for a correctly-packed frame, so no parameter reorder. */
static void canonical_to_priority(const uint8_t in9[9], uint8_t d[49])
{
    uint32_t A = ((uint32_t)in9[0] << 16) | ((uint32_t)in9[1] << 8) | in9[2];
    uint32_t B = (((uint32_t)in9[3] << 16) | ((uint32_t)in9[4] << 8) | in9[5]) >> 1;
    uint32_t C = (((uint32_t)in9[5] & 1u) << 24) | ((uint32_t)in9[6] << 16)
               | ((uint32_t)in9[7] << 8) | in9[8];
    uint32_t c0 = A >> 12;
    uint32_t c1 = (B ^ prng_mask_23(c0)) >> 11;
    for (int i = 0; i < 12; i++) d[i]      = (c0 >> (11 - i)) & 1u;
    for (int i = 0; i < 12; i++) d[12 + i] = (c1 >> (11 - i)) & 1u;
    for (int i = 0; i < 25; i++) d[24 + i] = (C  >> (24 - i)) & 1u;
}

/* 49 priority-order bits -> correct canonical 72-bit frame, re-protected. */
static void priority_to_canonical(const uint8_t d[49], uint8_t out9[9])
{
    uint32_t nc0 = 0, nc1 = 0, nC = 0;
    for (int i = 0; i < 12; i++) nc0 = (nc0 << 1) | d[i];
    for (int i = 0; i < 12; i++) nc1 = (nc1 << 1) | d[12 + i];
    for (int i = 0; i < 25; i++) nC  = (nC  << 1) | d[24 + i];
    uint32_t A2 = golay24_encode(nc0);
    uint32_t B2 = (golay24_encode(nc1) >> 1) ^ prng_mask_23(nc0);

    uint8_t o[9];
    o[0] = (uint8_t)(A2 >> 16); o[1] = (uint8_t)(A2 >> 8); o[2] = (uint8_t)A2;
    o[3] = (uint8_t)(B2 >> 15); o[4] = (uint8_t)(B2 >> 7);
    o[5] = (uint8_t)(((B2 & 0x7Fu) << 1) | ((nC >> 24) & 1u));
    o[6] = (uint8_t)(nC >> 16); o[7] = (uint8_t)(nC >> 8); o[8] = (uint8_t)nC;
    memcpy(out9, o, 9);
}

/* ---- 49-bit packing (MSB-first, low 7 bits of last byte zero) ---------- */

static void pack49(const uint8_t d[49], uint8_t voice7[7])
{
    memset(voice7, 0, 7);
    for (int i = 0; i < 49; i++)
        if (d[i]) voice7[i >> 3] |= (uint8_t)(0x80u >> (i & 7));
}

static void unpack49(const uint8_t voice7[7], uint8_t d[49])
{
    for (int i = 0; i < 49; i++)
        d[i] = (voice7[i >> 3] >> (7 - (i & 7))) & 1u;
}

/* ---- Public API -------------------------------------------------------- */

void dmr_prio_fix_stock_frame(const uint8_t in9[9], uint8_t out9[9])
{
    uint8_t d[49];
    stock_to_priority(in9, d);
    priority_to_canonical(d, out9);
}

void dmr_prio_stock_to_voice49(const uint8_t stock9[9], uint8_t voice7[7])
{
    uint8_t d[49];
    stock_to_priority(stock9, d);
    pack49(d, voice7);
}

void dmr_prio_canonical_to_voice49(const uint8_t canonical9[9], uint8_t voice7[7])
{
    uint8_t d[49];
    canonical_to_priority(canonical9, d);
    pack49(d, voice7);
}

void dmr_prio_voice49_to_canonical(const uint8_t voice7[7], uint8_t canonical9[9])
{
    uint8_t d[49];
    unpack49(voice7, d);
    priority_to_canonical(d, canonical9);
}
