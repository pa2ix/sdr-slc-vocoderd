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

/* swvoc.c — see swvoc.h. A thin, stateful wrapper over mbelib-neo 2.x. */
#include "swvoc.h"
#include <mbelib-neo/mbelib.h>
#include <stdlib.h>
#include <string.h>

/* ---- shared -------------------------------------------------------------- */

static void apply_gain(int16_t *pcm, float g)
{
    if (g == 1.0f) return;
    for (int i = 0; i < 160; i++) {
        float v = (float)pcm[i] * g;
        pcm[i] = (int16_t)(v > 32767.f ? 32767 : v < -32768.f ? -32768 : (int)(v + (v >= 0 ? 0.5f : -0.5f)));
    }
}

/* Map mbelib-neo's per-frame result onto our classes. Order matters: a
 * muted frame is also a repeat, and an invalid tone is flagged as both tone
 * and erasure. */
static swvoc_class_t classify(const mbe_process_result *r)
{
    if (r->flags & MBE_PROCESS_FLAG_MUTE)    return SWVOC_MUTED;
    if (r->flags & (MBE_PROCESS_FLAG_REPEAT | MBE_PROCESS_FLAG_ERASURE)) return SWVOC_CONCEALED;
    if (r->flags & MBE_PROCESS_FLAG_TONE)    return SWVOC_TONE;
    if (r->flags & MBE_PROCESS_FLAG_SILENCE) return SWVOC_SILENCE;
    return SWVOC_VOICE;
}

/* ---- decoder ------------------------------------------------------------- */

struct swdec {
    swvoc_family_t fam;
    mbe_parms cur, prev, prev_enh;
    char last_d[49];          /* parameter bits of the last frame, for concealment */
    int erasure_run;          /* consecutive RTP erasures                         */
    float gain;
};

swdec_t *swdec_create(swvoc_family_t fam)
{
    swdec_t *d = calloc(1, sizeof *d);
    if (!d) return NULL;
    d->fam = fam;
    d->gain = 1.0f;
    swdec_reset(d);
    return d;
}

void swdec_destroy(swdec_t *d) { free(d); }

void swdec_reset(swdec_t *d)
{
    if (!d) return;
    mbe_initMbeParms(&d->cur, &d->prev, &d->prev_enh);
    memset(d->last_d, 0, sizeof d->last_d);
    d->erasure_run = 0;
}

void swdec_set_gain(swdec_t *d, float gain) { if (d) d->gain = gain; }

static int finish(swdec_t *d, int rc, const mbe_process_result *r, int16_t pcm[160], swvoc_info_t *info)
{
    if (rc < 0) return -1;
    apply_gain(pcm, d->gain);
    if (info) { info->cls = classify(r); info->errors = r->total_errors; }
    return 0;
}

int swdec_frame(swdec_t *d, const char fr[4][24], int16_t pcm[160], swvoc_info_t *info)
{
    if (!d) return -1;
    mbe_process_result r;
    mbe_initProcessResult(&r);
    char ambe_d[49];
    int rc = (d->fam == SWVOC_AMBE2450)
        ? mbe_processAmbe3600x2450Frame((short *)pcm, &r, fr, ambe_d, &d->cur, &d->prev, &d->prev_enh)
        : mbe_processAmbe3600x2400Frame((short *)pcm, &r, fr, ambe_d, &d->cur, &d->prev, &d->prev_enh);
    if (rc >= 0) memcpy(d->last_d, ambe_d, sizeof ambe_d);
    d->erasure_run = 0;
    return finish(d, rc, &r, pcm, info);
}

/* Parameter-only decode with an explicit error context, as mbelib-neo's
 * Data API expects ("seed total_errors and any valid C0 context"). */
static int decode_data(swdec_t *d, const char ambe_d[49], int c0_errors, int16_t pcm[160], swvoc_info_t *info)
{
    mbe_process_result r;
    mbe_initProcessResult(&r);
    r.c0_errors = c0_errors;
    r.total_errors = c0_errors;
    r.flags |= MBE_PROCESS_FLAG_C0_VALID;
    int rc = (d->fam == SWVOC_AMBE2450)
        ? mbe_processAmbe2450Data((short *)pcm, &r, ambe_d, &d->cur, &d->prev, &d->prev_enh)
        : mbe_processAmbe2400Data((short *)pcm, &r, ambe_d, &d->cur, &d->prev, &d->prev_enh);
    return finish(d, rc, &r, pcm, info);
}

int swdec_params(swdec_t *d, const char ambe_d[49], int16_t pcm[160], swvoc_info_t *info)
{
    if (!d) return -1;
    memcpy(d->last_d, ambe_d, 49);
    d->erasure_run = 0;
    return decode_data(d, ambe_d, 0, pcm, info);
}

int swdec_erasure(swdec_t *d, int16_t pcm[160], swvoc_info_t *info)
{
    if (!d) return -1;
    /* Four C0 errors is past what Golay(24,12) can correct, so both families
     * treat the frame as invalid: AMBE+2 repeats on C0 >= 4 (TIA-102.BABA-1
     * 5.6) and D-STAR on more than 3 errors. The repeat counter then turns
     * a run of erasures into muting, exactly as for frames lost on air. */
    int rc = decode_data(d, d->last_d, 4, pcm, info);
    if (rc < 0) return rc;
    /* Repeat at most three times, then mute. AMBE+2 does this by itself;
     * mbelib-neo's D-STAR path resets its repeat counter on some frame
     * types, so the rule is enforced here for both families. The decoder
     * state still advanced above, so the next real frame resumes normally. */
    if (++d->erasure_run > 3) {
        memset(pcm, 0, 160 * sizeof pcm[0]);
        if (info) info->cls = SWVOC_MUTED;
    }
    if (info) info->errors = -1;
    return 0;
}

/* ---- encoder ------------------------------------------------------------- */

struct swenc {
    swvoc_family_t fam;
    mbe_ambe2450_encoder *e2450;
    mbe_ambe2400_encoder *e2400;
    mbe_parms cur, prev, scratch;
    float gain;
};

swenc_t *swenc_create(swvoc_family_t fam)
{
    swenc_t *e = calloc(1, sizeof *e);
    if (!e) return NULL;
    e->fam = fam;
    e->gain = 1.0f;
    if (fam == SWVOC_AMBE2450) e->e2450 = mbe_ambe2450EncoderAlloc();
    else                       e->e2400 = mbe_ambe2400EncoderAlloc();
    if (!e->e2450 && !e->e2400) { free(e); return NULL; }
    mbe_initMbeParms(&e->cur, &e->prev, &e->scratch);
    return e;
}

void swenc_destroy(swenc_t *e)
{
    if (!e) return;
    mbe_ambe2450EncoderFree(e->e2450);
    mbe_ambe2400EncoderFree(e->e2400);
    free(e);
}

void swenc_reset(swenc_t *e)
{
    if (!e) return;
    if (e->e2450) mbe_ambe2450EncoderReset(e->e2450);
    if (e->e2400) mbe_ambe2400EncoderReset(e->e2400);
    mbe_initMbeParms(&e->cur, &e->prev, &e->scratch);
}

void swenc_set_gain(swenc_t *e, float gain) { if (e) e->gain = gain; }

int swenc_params(swenc_t *e, const int16_t pcm[160], char ambe_d[49])
{
    if (!e) return -1;
    int16_t in[160];
    memcpy(in, pcm, sizeof in);
    apply_gain(in, e->gain);
    int rc = e->e2450
        ? mbe_encodeAmbe2450ParmsShort(e->e2450, (const short *)in, ambe_d, &e->cur, &e->prev)
        : mbe_encodeAmbe2400ParmsShort(e->e2400, (const short *)in, ambe_d, &e->cur, &e->prev);
    if (rc < 0) return -1;
    mbe_moveMbeParms(&e->cur, &e->prev);   /* advance prediction, per the API contract */
    return 0;
}

int swenc_frame(swenc_t *e, const int16_t pcm[160], char fr[4][24])
{
    char ambe_d[49];
    if (swenc_params(e, pcm, ambe_d) < 0) return -1;
    int rc = (e->fam == SWVOC_AMBE2450) ? mbe_encodeAmbe3600x2450Frame(ambe_d, fr)
                                        : mbe_encodeAmbe3600x2400Frame(ambe_d, fr);
    return rc < 0 ? -1 : 0;
}
