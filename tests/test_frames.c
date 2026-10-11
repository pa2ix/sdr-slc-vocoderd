/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * sdr-slc-vocoderd — digital voice vocoder service for the SDR-SLC protocol suite.
 * Copyright (C) 2026 Ivo van Ling (PA2IX)
 *
 * test_frames — proves the wire <-> mbelib-neo mappings in frames.c.
 *
 * The reference is sdr-slc-codecd's own dmr_air.c and dmr_prio.c (copied
 * unchanged into tests/ref/), which are proven on air: codecd's on-air
 * interleave against a real MD-380 capture, and its 49<->72 conversion as the
 * YSF path. If frames.c agrees with them bit for bit, the new daemon puts
 * exactly the same bits on the wire as the old one.
 */
#include "frames.h"
#include "ref/dmr_air.h"
#include "ref/dmr_prio.h"
#include <mbelib-neo/mbelib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

static uint32_t rng = 0x12345678u;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static void rand_d(char d[49]) { for (int i = 0; i < 49; i++) d[i] = (char)(rnd() & 1); }
static void rand_bytes(uint8_t *b, int n) { for (int i = 0; i < n; i++) b[i] = (uint8_t)rnd(); }

int main(void)
{
    const int N = 20000;

    /* 1. on-air <-> 4x24 planes is a bijection. */
    for (int k = 0; k < N; k++) {
        uint8_t a[9], b[9]; char fr[4][24];
        rand_bytes(a, 9);
        frames_dmr_air_to_fr(a, fr);
        frames_fr_to_dmr_air((const char (*)[24])fr, b);
        CHECK(!memcmp(a, b, 9), "dmr air->fr->air not identity");
    }

    /* 2. air <-> canonical matches codecd's dmr_air exactly, both ways. */
    for (int k = 0; k < N; k++) {
        uint8_t a[9], mine[9], ref[9];
        rand_bytes(a, 9);
        frames_dmr_air_to_canonical(a, mine); dmr_air_deinterleave(a, ref);
        CHECK(!memcmp(mine, ref, 9), "air->canonical differs from codecd");
        frames_dmr_canonical_to_air(a, mine); dmr_air_interleave(a, ref);
        CHECK(!memcmp(mine, ref, 9), "canonical->air differs from codecd");
    }

    /* 3. 49 -> 72 on-air equals codecd's voice49 -> canonical -> interleave,
     *    and 72 -> 49 equals codecd's deinterleave -> canonical_to_voice49. */
    for (int k = 0; k < N; k++) {
        char d[49], d2[49]; uint8_t v7[7], v7r[7], air[9], can[9], ref_air[9];
        rand_d(d);
        frames_d_to_voice49(d, v7);
        frames_voice49_to_d(v7, d2);
        CHECK(!memcmp(d, d2, 49), "voice49 pack/unpack");
        CHECK((v7[6] & 0x7F) == 0, "voice49 padding bits not zero");

        CHECK(frames_d_to_dmr_air(d, air) == 0, "d->air");
        dmr_prio_voice49_to_canonical(v7, can);
        dmr_air_interleave(can, ref_air);
        CHECK(!memcmp(air, ref_air, 9), "49->72 differs from codecd (k=%d)", k);

        int errs = -1;
        CHECK(frames_dmr_air_to_d(air, d2, &errs) == 0 && errs == 0, "clean frame reported %d errors", errs);
        CHECK(!memcmp(d, d2, 49), "72->49 round trip");
        dmr_air_deinterleave(air, can);
        dmr_prio_canonical_to_voice49(can, v7r);
        CHECK(!memcmp(v7, v7r, 7), "72->49 differs from codecd");
    }

    /* 4. FEC really runs on our planes: one bit error in C0 is corrected. */
    for (int k = 0; k < 2000; k++) {
        char d[49], d2[49], fr[4][24]; uint8_t air[9];
        rand_d(d);
        mbe_encodeAmbe3600x2450Frame(d, fr);
        fr[0][rnd() % 24] ^= 1;
        frames_fr_to_dmr_air((const char (*)[24])fr, air);
        int errs = -1;
        frames_dmr_air_to_d(air, d2, &errs);
        CHECK(errs == 1 && !memcmp(d, d2, 49), "single C0 error not corrected (errs=%d)", errs);
    }

    /* 5. D-STAR DV bytes <-> planes is a bijection. */
    for (int k = 0; k < N; k++) {
        uint8_t a[9], b[9]; char fr[4][24];
        rand_bytes(a, 9);
        CHECK(frames_dstar_to_fr(a, fr) == 0, "dstar unpack");
        CHECK(frames_fr_to_dstar((const char (*)[24])fr, b) == 0, "dstar pack");
        CHECK(!memcmp(a, b, 9), "dstar bytes->fr->bytes not identity");
    }

    if (fails) { fprintf(stderr, "test_frames: %u failure(s)\n", fails); return 1; }
    puts("test_frames OK (DMR/YSF bit-identical to sdr-slc-codecd, D-STAR bijective)");
    return 0;
}
