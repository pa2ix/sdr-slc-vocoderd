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

/* swvoc.h — the software vocoder: one stateful mbelib-neo decoder or encoder
 * per stream, for both AMBE families the daemon serves.
 *
 *   SWVOC_AMBE2450  AMBE+2 3600x2450: DMR, YSF (and NXDN, dPMR, P25 Phase 2)
 *   SWVOC_AMBE2400  AMBE   3600x2400: D-STAR
 *
 * Frames go in and out as mbelib-neo's own representations (frames.h has the
 * wire mappings). Besides PCM, each decoded frame is CLASSIFIED, and that
 * classification is what the chip arbiter uses to decide whether a receive
 * stream is carrying a real transmission: an idle timeslot or a squelch-open
 * channel produces frames that fail FEC, decode as erasures or are repeated
 * and muted, never as a run of clean voice frames.
 *
 * Output level: mbelib-neo synthesises at the level DVSI's own decoders
 * produce on the same bits, so software and AMBE-3000 audio match without the
 * -17.5 dB correction sdr-slc-codecd needed for OpenDMR. */
#ifndef SLC_SWVOC_H
#define SLC_SWVOC_H
#include <stdint.h>

typedef enum { SWVOC_AMBE2450 = 0, SWVOC_AMBE2400 = 1 } swvoc_family_t;

/* What a decoded frame was. */
typedef enum {
    SWVOC_VOICE = 0,   /* a clean voice frame: speech (or low-level voice)    */
    SWVOC_TONE,        /* a DTMF / call-progress / single-tone frame          */
    SWVOC_SILENCE,     /* an AMBE+2 silence frame (b0 124/125)                */
    SWVOC_CONCEALED,   /* repeated: erasure, FEC failure, or an RTP erasure   */
    SWVOC_MUTED        /* muted: too many errors / repeats in a row           */
} swvoc_class_t;

typedef struct {
    swvoc_class_t cls;
    int errors;        /* corrected bit errors (C0 + protected), or -1        */
} swvoc_info_t;

/* Is this class evidence of an active transmission? Voice and tones are;
 * silence, concealment and muting are not. */
static inline int swvoc_is_activity(swvoc_class_t c) { return c == SWVOC_VOICE || c == SWVOC_TONE; }

/* Evidence of a real transmission over a sliding window of frames.
 *
 * One frame proves little: Golay codes "correct" almost any random word into
 * a valid codeword, so 92% of random DMR frames decode as voice frames, just
 * with 2-4 corrected errors each (clean frames carry 0). Summed over a short
 * window the two separate cleanly. Measured with mbelib-neo 2.4 over 10^6
 * random frames and a synthetic speech signal with random bit errors:
 *
 *                                 sum over 6 frames (120 ms)
 *                                   <= 6        <= 8
 *     random frames (noise)       2e-6        1e-4     per frame
 *     speech at 1% bit errors     99.7%       100%     of windows
 *     speech at 2% bit errors     92.7%       99.0%
 *
 * SWVOC_WIN_CLAIM is what a receive stream needs to CLAIM the AMBE-3000:
 * pure noise passes about once every three hours. SWVOC_WIN_HOLD is the
 * looser test that keeps an owner's claim alive, so a fading signal does not
 * lose the chip mid-over. A window qualifies only if every frame in it is
 * voice or tone. Frames without an error count (YSF, whose FEC the client
 * already removed) count as zero errors. */
#define SWVOC_WIN        6
#define SWVOC_WIN_CLAIM  6
#define SWVOC_WIN_HOLD   8

typedef struct { int err[SWVOC_WIN]; unsigned char act[SWVOC_WIN]; int n, pos; } swvoc_window_t;

static inline void swvoc_window_reset(swvoc_window_t *w) { w->n = 0; w->pos = 0; }
static inline void swvoc_window_push(swvoc_window_t *w, const swvoc_info_t *in)
{
    w->act[w->pos] = (unsigned char)swvoc_is_activity(in->cls);
    w->err[w->pos] = in->errors < 0 ? 0 : in->errors;
    w->pos = (w->pos + 1) % SWVOC_WIN;
    if (w->n < SWVOC_WIN) w->n++;
}
/* 1 if the last SWVOC_WIN frames are all activity with <= max_errors total. */
static inline int swvoc_window_ok(const swvoc_window_t *w, int max_errors)
{
    if (w->n < SWVOC_WIN) return 0;
    int sum = 0;
    for (int i = 0; i < SWVOC_WIN; i++) { if (!w->act[i]) return 0; sum += w->err[i]; }
    return sum <= max_errors;
}

typedef struct swdec swdec_t;
swdec_t *swdec_create(swvoc_family_t fam);
void     swdec_destroy(swdec_t *d);
void     swdec_reset(swdec_t *d);
/* Linear gain applied to the synthesised PCM (default 1.0). */
void     swdec_set_gain(swdec_t *d, float gain);

/* Decode one 72-bit channel frame (FEC applied here). 0 or -1. */
int swdec_frame(swdec_t *d, const char fr[4][24], int16_t pcm[160], swvoc_info_t *info);
/* Decode 49 AMBE+2 parameter bits that carry no AMBE FEC (YSF V/D mode 2,
 * whose own channel coding the client already removed). 0 or -1. */
int swdec_params(swdec_t *d, const char ambe_d[49], int16_t pcm[160], swvoc_info_t *info);
/* Conceal a lost frame (RTP erasure): repeat per TIA-102.BABA-1 5.6, which
 * turns into muting after three repeats. 0 or -1. */
int swdec_erasure(swdec_t *d, int16_t pcm[160], swvoc_info_t *info);

typedef struct swenc swenc_t;
swenc_t *swenc_create(swvoc_family_t fam);
void     swenc_destroy(swenc_t *e);
void     swenc_reset(swenc_t *e);
/* Linear gain applied to the PCM before analysis (default 1.0). */
void     swenc_set_gain(swenc_t *e, float gain);
/* Encode 20 ms of PCM into 49 parameter bits. 0 or -1. */
int swenc_params(swenc_t *e, const int16_t pcm[160], char ambe_d[49]);
/* Encode 20 ms of PCM into a complete 72-bit frame (4x24 planes, FEC on). */
int swenc_frame(swenc_t *e, const int16_t pcm[160], char fr[4][24]);

#endif
