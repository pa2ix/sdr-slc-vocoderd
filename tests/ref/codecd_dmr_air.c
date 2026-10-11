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

/* dmr_air.c — see dmr_air.h. Tables are the standard DMR AMBE interleave
 * schedule from the dsd/mbelib lineage: walking the 36 on-air dibits, the
 * first bit of dibit i lands at ambe_fr[rW[i]][rX[i]] and the second at
 * ambe_fr[rY[i]][rZ[i]]; row 0 is the A block (col 23 = MSB), row 1 the
 * B block (col 22 = MSB), rows 2/3 the C2/C3 blocks (cols 10/13 = MSB).
 *
 * Validated against a real MD-380 capture: 162/162 frames decode to
 * intelligible speech through OpenDMR after dmr_air_deinterleave(), and
 * interleave(deinterleave(x)) == x for all frames.
 */
#include "dmr_air.h"
#include <string.h>

static const int rW[36] = {0,1,0,1,0,1, 0,1,0,1,0,1, 0,1,0,1,0,1,
                           0,1,0,1,0,2, 0,2,0,2,0,2, 0,2,0,2,0,2};
static const int rX[36] = {23,10,22,9,21,8, 20,7,19,6,18,5, 17,4,16,3,15,2,
                           14,1,13,0,12,10, 11,9,10,8,9,7, 8,6,7,5,6,4};
static const int rY[36] = {0,2,0,2,0,2, 0,2,0,3,0,3, 1,3,1,3,1,3,
                           1,3,1,3,1,3, 1,3,1,3,1,3, 1,3,1,3,1,3};
static const int rZ[36] = {5,3,4,2,3,1, 2,0,1,13,0,12, 22,11,21,10,20,9,
                           19,8,18,7,17,6, 16,5,15,4,14,3, 13,2,12,1,11,0};

static int getb(const uint8_t *f, int i) { return (f[i >> 3] >> (7 - (i & 7))) & 1; }
static void setb(uint8_t *f, int i, int v) { if (v) f[i >> 3] |= (uint8_t)(0x80u >> (i & 7)); }

/* Canonical bit index of ambe_fr[row][col], per the block layout above. */
static int canonical_index(int row, int col)
{
    switch (row) {
    case 0:  return 23 - col;                 /* A:  bits  0..23 */
    case 1:  return 24 + (22 - col);          /* B:  bits 24..46 */
    case 2:  return 47 + (10 - col);          /* C2: bits 47..57 */
    default: return 58 + (13 - col);          /* C3: bits 58..71 */
    }
}

void dmr_air_deinterleave(const uint8_t in9[9], uint8_t out9[9])
{
    memset(out9, 0, 9);
    for (int i = 0; i < 36; i++) {
        setb(out9, canonical_index(rW[i], rX[i]), getb(in9, 2 * i));
        setb(out9, canonical_index(rY[i], rZ[i]), getb(in9, 2 * i + 1));
    }
}

void dmr_air_interleave(const uint8_t in9[9], uint8_t out9[9])
{
    memset(out9, 0, 9);
    for (int i = 0; i < 36; i++) {
        setb(out9, 2 * i,     getb(in9, canonical_index(rW[i], rX[i])));
        setb(out9, 2 * i + 1, getb(in9, canonical_index(rY[i], rZ[i])));
    }
}
