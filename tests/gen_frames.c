/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * sdr-slc-vocoderd — digital voice vocoder service for the SDR-SLC protocol suite.
 * Copyright (C) 2026 Ivo van Ling (PA2IX)
 *
 * gen_frames — test signal generator for the end-to-end test.
 *   gen_frames FMT voice N   N wire frames of a speech-like signal (FMT dmr|ysf|dstar)
 *   gen_frames FMT noise N   N random wire frames (an idle slot / open squelch)
 *   gen_frames pcm voice N   N frames of the speech-like signal as s16be PCM
 * Frames are written back to back to stdout.
 */
#define _DEFAULT_SOURCE
#include "frames.h"
#include "swvoc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void speech(int16_t pcm[160], long frame)
{
    for (int i = 0; i < 160; i++) {
        double t = (frame * 160 + i) / 8000.0, f0 = 120 + 30 * sin(2 * M_PI * 0.7 * t);
        double v = sin(2 * M_PI * f0 * t) + 0.6 * sin(2 * M_PI * 2.3 * f0 * t) + 0.3 * sin(2 * M_PI * 5.1 * f0 * t);
        pcm[i] = (int16_t)lrint(6000 * v * (0.6 + 0.4 * sin(2 * M_PI * 3 * t)));
    }
}

int main(int argc, char **argv)
{
    if (argc != 4) { fprintf(stderr, "usage: %s dmr|ysf|dstar|pcm voice|noise N\n", argv[0]); return 2; }
    const char *fmt = argv[1];
    int noise = !strcmp(argv[2], "noise");
    long n = atol(argv[3]);
    swenc_t *e = swenc_create(!strcmp(fmt, "dstar") ? SWVOC_AMBE2400 : SWVOC_AMBE2450);
    uint32_t r = 12345;
    for (long k = 0; k < n; k++) {
        int16_t pcm[160]; uint8_t w[320]; size_t len = 9; char fr[4][24], d[49];
        speech(pcm, k);
        if (!strcmp(fmt, "pcm")) {
            for (int i = 0; i < 160; i++) { w[2 * i] = (uint8_t)((uint16_t)pcm[i] >> 8); w[2 * i + 1] = (uint8_t)pcm[i]; }
            len = 320;
        } else if (noise) {
            for (int i = 0; i < 9; i++) { r ^= r << 13; r ^= r >> 17; r ^= r << 5; w[i] = (uint8_t)r; }
            if (!strcmp(fmt, "ysf")) { len = 7; w[6] &= 0x80; }
        } else if (!strcmp(fmt, "ysf")) {
            swenc_params(e, pcm, d); frames_d_to_voice49(d, w); len = 7;
        } else {
            swenc_frame(e, pcm, fr);
            if (!strcmp(fmt, "dstar")) frames_fr_to_dstar((const char (*)[24])fr, w);
            else frames_fr_to_dmr_air((const char (*)[24])fr, w);
        }
        fwrite(w, 1, len, stdout);
    }
    return 0;
}
