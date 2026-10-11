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

/* frames.c — see frames.h.
 *
 * The DMR interleave schedule is the standard dsd/mbelib one: walking the 36
 * on-air dibits, the first bit of dibit i lands at fr[rW[i]][rX[i]] and the
 * second at fr[rY[i]][rZ[i]]. These are the tables sdr-slc-codecd validated
 * against a real MD-380 capture (162/162 frames), and they are identical to
 * the ones mbelib-neo uses to import DVSI's rate-33 test vectors. */
#include "frames.h"
#include <mbelib-neo/mbelib.h>
#include <string.h>

static const unsigned char rW[36] = {0,1,0,1,0,1, 0,1,0,1,0,1, 0,1,0,1,0,1,
                                     0,1,0,1,0,2, 0,2,0,2,0,2, 0,2,0,2,0,2};
static const unsigned char rX[36] = {23,10,22,9,21,8, 20,7,19,6,18,5, 17,4,16,3,15,2,
                                     14,1,13,0,12,10, 11,9,10,8,9,7, 8,6,7,5,6,4};
static const unsigned char rY[36] = {0,2,0,2,0,2, 0,2,0,3,0,3, 1,3,1,3,1,3,
                                     1,3,1,3,1,3, 1,3,1,3,1,3, 1,3,1,3,1,3};
static const unsigned char rZ[36] = {5,3,4,2,3,1, 2,0,1,13,0,12, 22,11,21,10,20,9,
                                     19,8,18,7,17,6, 16,5,15,4,14,3, 13,2,12,1,11,0};

static int getb(const uint8_t *f, int i) { return (f[i >> 3] >> (7 - (i & 7))) & 1; }
static void setb(uint8_t *f, int i, int v) { if (v) f[i >> 3] |= (uint8_t)(0x80u >> (i & 7)); }

void frames_dmr_air_to_fr(const uint8_t air9[9], char fr[4][24])
{
    memset(fr, 0, 4 * 24);
    for (int i = 0; i < 36; i++) {
        fr[rW[i]][rX[i]] = (char)getb(air9, 2 * i);
        fr[rY[i]][rZ[i]] = (char)getb(air9, 2 * i + 1);
    }
}

void frames_fr_to_dmr_air(const char fr[4][24], uint8_t air9[9])
{
    memset(air9, 0, 9);
    for (int i = 0; i < 36; i++) {
        setb(air9, 2 * i,     fr[rW[i]][rX[i]] & 1);
        setb(air9, 2 * i + 1, fr[rY[i]][rZ[i]] & 1);
    }
}

/* Canonical bit index of fr[row][col]: A bits 0..23, B 24..46, C2 47..57,
 * C3 58..71, each block MSB first. */
static int canonical_index(int row, int col)
{
    switch (row) {
    case 0:  return 23 - col;
    case 1:  return 24 + (22 - col);
    case 2:  return 47 + (10 - col);
    default: return 58 + (13 - col);
    }
}

void frames_dmr_air_to_canonical(const uint8_t air9[9], uint8_t can9[9])
{
    memset(can9, 0, 9);
    for (int i = 0; i < 36; i++) {
        setb(can9, canonical_index(rW[i], rX[i]), getb(air9, 2 * i));
        setb(can9, canonical_index(rY[i], rZ[i]), getb(air9, 2 * i + 1));
    }
}

void frames_dmr_canonical_to_air(const uint8_t can9[9], uint8_t air9[9])
{
    memset(air9, 0, 9);
    for (int i = 0; i < 36; i++) {
        setb(air9, 2 * i,     getb(can9, canonical_index(rW[i], rX[i])));
        setb(air9, 2 * i + 1, getb(can9, canonical_index(rY[i], rZ[i])));
    }
}

void frames_voice49_to_d(const uint8_t v7[7], char d[49])
{
    for (int i = 0; i < 49; i++) d[i] = (char)getb(v7, i);
}

void frames_d_to_voice49(const char d[49], uint8_t v7[7])
{
    memset(v7, 0, 7);
    for (int i = 0; i < 49; i++) setb(v7, i, d[i] & 1);
}

int frames_d_to_dmr_air(const char d[49], uint8_t air9[9])
{
    char fr[4][24];
    if (mbe_encodeAmbe3600x2450Frame(d, fr) < 0) return -1;
    frames_fr_to_dmr_air((const char (*)[24])fr, air9);
    return 0;
}

int frames_dmr_air_to_d(const uint8_t air9[9], char d[49], int *errs)
{
    char fr[4][24];
    frames_dmr_air_to_fr(air9, fr);
    int e = mbe_decodeAmbe3600x2450Frame((const char (*)[24])fr, d, NULL);
    if (e < 0) return -1;
    if (errs) *errs = e;
    return 0;
}

int frames_dstar_to_fr(const uint8_t dv9[9], char fr[4][24])
{
    return mbe_decodeDStarDVData(dv9, fr) < 0 ? -1 : 0;
}

int frames_fr_to_dstar(const char fr[4][24], uint8_t dv9[9])
{
    return mbe_encodeDStarDVData(fr, dv9) < 0 ? -1 : 0;
}
