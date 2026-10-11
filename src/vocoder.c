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

/* vocoder.c — see vocoder.h for the policy. This file is the mechanism.
 *
 * Locking. Two mutexes, always taken in the order io -> arb:
 *   g_arb  guards the ownership state below. Held only for a few lines of
 *          bookkeeping, never across chip I/O, so a stream that does not own
 *          the chip never waits for the chip.
 *   g_io   serialises chip I/O and the chip handle's lifetime. Only the owner
 *          takes it per frame. A preempting transmit waits at most for the
 *          frame the previous owner has in flight (normally ~9 ms).
 * Before each transaction the owner re-checks, under both locks, that it is
 * still the owner of the same healthy chip, so a stream that lost the chip
 * between deciding and doing can never drive it. */
#define _POSIX_C_SOURCE 200809L
#include "vocoder.h"
#include "ambe3k.h"
#include "frames.h"
#include "swvoc.h"
#include <errno.h>
#include <math.h>
#include <mbelib-neo/mbelib.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Chip op return for "the device is gone", matching AMBE3K_EGONE. */
#define CHIP_GONE (-2)
/* Consecutive failed transactions after which a present chip is reopened. */
#define CHIP_MAX_FAILS 3

static struct voc_config g_cfg;

/* ---- clock --------------------------------------------------------------- */
static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* ---- chip ops: the AMBE-3000 driver (default) ---------------------------- */
static char g_cand_sig[512];
static int  g_cand_first = 1;

static void *drv_open_any(char *name, unsigned long name_len)
{
    ambe3k_t *h = NULL;
    char path[256] = {0}, prodid[AMBE3K_PRODID_MAX] = {0}, hint[256] = {0};
    if (g_cfg.hw_dev) {
        h = ambe3k_open(g_cfg.hw_dev);
        if (h) snprintf(path, sizeof path, "%s", g_cfg.hw_dev);
    } else {
        ambe3k_status_t st = ambe3k_autodetect(&h, path, sizeof path, prodid, sizeof prodid, hint, sizeof hint);
        if (st != AMBE3K_FOUND && hint[0]) fprintf(stderr, "vocoder: %s\n", hint);
    }
    if (!h) return NULL;
    if ((g_cfg.hw_in_gain_db || g_cfg.hw_out_gain_db) &&
        ambe3k_set_gain(h, g_cfg.hw_in_gain_db, g_cfg.hw_out_gain_db) == 0)
        fprintf(stderr, "vocoder: AMBE-3000 gain set (in %+d dB, out %+d dB)\n", g_cfg.hw_in_gain_db, g_cfg.hw_out_gain_db);
    snprintf(name, name_len, "%s on %s", ambe3k_prodid(h), ambe3k_path(h));
    return h;
}
static void drv_close(void *h) { ambe3k_close(h); }
static int drv_set_rate(void *h, int dstar) { return ambe3k_set_rate(h, dstar ? AMBE3K_RATE_DSTAR : AMBE3K_RATE_DMR); }
static int drv_decode(void *h, const uint8_t in9[9], int16_t pcm[160]) { return ambe3k_decode(h, in9, pcm); }
static int drv_encode(void *h, const int16_t pcm[160], uint8_t out9[9]) { return ambe3k_encode(h, pcm, out9); }
static int drv_changed(void)
{
    if (g_cfg.hw_dev) return 0;              /* fixed device: the retry timer drives it */
    char sig[512];
    ambe3k_candidates(sig, sizeof sig);
    int changed = g_cand_first || strcmp(sig, g_cand_sig) != 0;
    if (changed) { snprintf(g_cand_sig, sizeof g_cand_sig, "%s", sig); g_cand_first = 0; }
    return changed;
}
static const struct voc_chip_ops DRV_OPS = { drv_open_any, drv_close, drv_set_rate, drv_decode, drv_encode, drv_changed };
static struct voc_chip_ops g_ops;
static int g_ops_set = 0;
void voc_set_chip_ops(const struct voc_chip_ops *ops) { g_ops = *ops; g_ops_set = 1; }

/* ---- shared state -------------------------------------------------------- */
static pthread_mutex_t g_io  = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_arb = PTHREAD_MUTEX_INITIALIZER;

static void   *g_chip = NULL;              /* io+arb to change, either to read */
static char    g_chip_name[192] = "";
static int     g_chip_down = 0;            /* arb: a failure asked for a reopen */
static int     g_fail_run = 0;             /* io: consecutive failed transactions */
static int     g_chip_dstar = -1;          /* io: rate the chip is set to, -1 unknown */

enum { PH_WARM, PH_XFADE, PH_LIVE };
static voc_stream_t *g_owner = NULL;       /* arb */
static int     g_owner_tx = 0;             /* arb */
static int64_t g_owner_last = 0;           /* arb: last evidence (rx) / voice (tx) */
static int     g_need_reset = 0;           /* arb: new owner, chip not yet reset */
static int     g_phase = PH_LIVE;          /* arb */
static int     g_warm_left = 0;            /* arb */

/* discovery thread */
static pthread_t       g_mgr;
static int             g_mgr_running = 0;
static int             g_mgr_stop = 0;
static pthread_mutex_t g_mgr_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_mgr_cond = PTHREAD_COND_INITIALIZER;

struct voc_stream {
    voc_fmt_t fmt;
    int decode;
    char id[40];
    swdec_t *dec;
    swenc_t *enc;
    swvoc_window_t win;
};

static int chip_usable_locked(void) { return g_chip && !g_chip_down; }

static int owner_idle_locked(int64_t now)
{
    if (!g_owner) return 1;
    return now - g_owner_last > (g_owner_tx ? g_cfg.tx_hang_ms : g_cfg.rx_hang_ms);
}

static const char *dir_name(const voc_stream_t *s) { return s->decode ? "RX" : "TX"; }

static void acquire_locked(voc_stream_t *s, int tx, int64_t now)
{
    if (g_cfg.verbose) {
        if (g_owner) fprintf(stderr, "[chip] %s %s takes the AMBE-3000 from %s %s\n", s->id, dir_name(s), g_owner->id, dir_name(g_owner));
        else         fprintf(stderr, "[chip] %s %s takes the AMBE-3000\n", s->id, dir_name(s));
    }
    g_owner = s;
    g_owner_tx = tx;
    g_owner_last = now;
    g_need_reset = 1;
    g_phase = tx ? PH_LIVE : (g_cfg.warmup_frames > 0 ? PH_WARM : PH_XFADE);
    g_warm_left = g_cfg.warmup_frames;
}

static void release_locked(voc_stream_t *s, const char *why)
{
    if (g_owner != s) return;
    if (g_cfg.verbose) fprintf(stderr, "[chip] %s %s releases the AMBE-3000 (%s)\n", s->id, dir_name(s), why);
    g_owner = NULL;
}

static void wake_manager(void)
{
    pthread_mutex_lock(&g_mgr_lock);
    pthread_cond_signal(&g_mgr_cond);
    pthread_mutex_unlock(&g_mgr_lock);
}

/* Begin a chip transaction for `s`. On success g_io is HELD and the chip
 * handle is returned, with *reset_rate = -1 (no reset needed) or the rate to
 * program first (0 DMR, 1 D-STAR). Returns NULL (g_io not held) if `s` no
 * longer owns a usable chip. */
static void *io_begin(voc_stream_t *s, int *reset_rate)
{
    pthread_mutex_lock(&g_io);
    pthread_mutex_lock(&g_arb);
    void *h = (g_owner == s && chip_usable_locked()) ? g_chip : NULL;
    *reset_rate = (h && g_need_reset) ? (s->fmt == VOC_FMT_DSTAR) : -1;
    pthread_mutex_unlock(&g_arb);
    if (!h) pthread_mutex_unlock(&g_io);
    return h;
}

/* End a transaction begun with io_begin (releases g_io). rc is the chip
 * result; failures are counted and a gone or repeatedly failing chip is
 * handed to the discovery thread to close and reopen. */
static void io_end(voc_stream_t *s, int rc, int did_reset)
{
    int down = 0;
    if (rc == 0) {
        g_fail_run = 0;
    } else if (rc == CHIP_GONE || ++g_fail_run >= CHIP_MAX_FAILS) {
        down = 1;
    }
    pthread_mutex_lock(&g_arb);
    if (rc == 0 && did_reset && g_owner == s) g_need_reset = 0;
    if (down && !g_chip_down) {
        g_chip_down = 1;
        fprintf(stderr, "vocoder: AMBE-3000 %s; all streams continue on %s while it is reopened\n",
                rc == CHIP_GONE ? "disappeared" : "stopped answering", voc_sw_version());
        if (g_owner) g_owner = NULL;
    }
    pthread_mutex_unlock(&g_arb);
    pthread_mutex_unlock(&g_io);
    if (down) wake_manager();
}

/* One complete chip transaction: reset if this is a new owner, then decode
 * (pcm_out != NULL) or encode. Returns 0 if the chip produced the frame. */
static int chip_xact(voc_stream_t *s, const uint8_t *in9, int16_t *pcm_out, const int16_t *pcm_in, uint8_t *out9)
{
    int reset_rate;
    void *h = io_begin(s, &reset_rate);
    if (!h) return -1;
    int rc = 0, did_reset = 0;
    if (reset_rate >= 0) {
        rc = g_ops.set_rate(h, reset_rate);   /* RATEP also re-initialises the vocoder */
        if (rc == 0) { did_reset = 1; g_chip_dstar = reset_rate; }
    }
    if (rc == 0) rc = pcm_out ? g_ops.decode(h, in9, pcm_out) : g_ops.encode(h, pcm_in, out9);
    io_end(s, rc, did_reset);
    return rc == 0 ? 0 : -1;
}

/* ---- discovery thread ---------------------------------------------------- */
static void *manager(void *arg)
{
    (void)arg;
    int64_t retry_at = 0, backoff = 2000;
    int first = 1;
    pthread_mutex_lock(&g_mgr_lock);
    while (!g_mgr_stop) {
        pthread_mutex_unlock(&g_mgr_lock);

        /* A failing or vanished chip: close it, then try again later. */
        pthread_mutex_lock(&g_io);
        pthread_mutex_lock(&g_arb);
        void *dead = (g_chip && g_chip_down) ? g_chip : NULL;
        if (dead) { g_chip = NULL; g_chip_down = 0; g_owner = NULL; g_chip_dstar = -1; g_fail_run = 0; g_chip_name[0] = 0; }
        pthread_mutex_unlock(&g_arb);
        pthread_mutex_unlock(&g_io);
        if (dead) {
            g_ops.close(dead);
            retry_at = now_ms() + backoff;
            if (backoff < 30000) backoff *= 2;
        }

        int have;
        pthread_mutex_lock(&g_arb); have = g_chip != NULL; pthread_mutex_unlock(&g_arb);
        if (!have && !g_cfg.disable_hw) {
            int64_t now = now_ms();
            int changed = g_ops.changed ? g_ops.changed() : 0;
            int timer = g_cfg.hw_dev ? (now >= retry_at) : (retry_at && now >= retry_at);
            if (first || changed || timer) {
                char name[192] = "";
                void *h = g_ops.open_any(name, sizeof name);
                if (h) {
                    pthread_mutex_lock(&g_io);
                    pthread_mutex_lock(&g_arb);
                    g_chip = h; g_chip_down = 0; g_fail_run = 0; g_chip_dstar = -1;
                    snprintf(g_chip_name, sizeof g_chip_name, "%s", name);
                    pthread_mutex_unlock(&g_arb);
                    pthread_mutex_unlock(&g_io);
                    fprintf(stderr, "vocoder: AMBE-3000 available (%s): it now serves the active talker, "
                                    "%s serves every other stream\n", name, voc_sw_version());
                    retry_at = 0; backoff = 2000;
                } else {
                    if (first) fprintf(stderr, "vocoder: no AMBE-3000 found; using %s for all streams "
                                               "(plug one in at any time, it is picked up automatically)\n", voc_sw_version());
                    retry_at = g_cfg.hw_dev ? now + 5000 : (retry_at ? now + backoff : 0);
                    if (retry_at && backoff < 30000) backoff *= 2;
                }
                first = 0;
            }
        }

        pthread_mutex_lock(&g_mgr_lock);
        if (g_mgr_stop) break;
        struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 500 * 1000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        pthread_cond_timedwait(&g_mgr_cond, &g_mgr_lock, &ts);
    }
    pthread_mutex_unlock(&g_mgr_lock);
    return NULL;
}

/* ---- init ---------------------------------------------------------------- */
void voc_config_defaults(struct voc_config *c)
{
    memset(c, 0, sizeof *c);
    c->sw_dec_gain = 1.0f;
    c->sw_enc_gain = 1.0f;
    c->tx_vad_dbfs = -50.0f;
    c->tx_hang_ms = 2000;
    c->rx_hang_ms = 300;
    c->warmup_frames = 3;
}

int voc_init(const struct voc_config *c)
{
    g_cfg = *c;
    if (!g_ops_set) g_ops = DRV_OPS;
    if (g_cfg.disable_hw) {
        fprintf(stderr, "vocoder: AMBE-3000 disabled (--no-hw); using %s for all streams\n", voc_sw_version());
        return 0;
    }
    g_mgr_stop = 0;
    if (pthread_create(&g_mgr, NULL, manager, NULL) != 0) return -1;
    g_mgr_running = 1;
    return 0;
}

void voc_shutdown(void)
{
    if (g_mgr_running) {
        pthread_mutex_lock(&g_mgr_lock);
        g_mgr_stop = 1;
        pthread_cond_signal(&g_mgr_cond);
        pthread_mutex_unlock(&g_mgr_lock);
        pthread_join(g_mgr, NULL);
        g_mgr_running = 0;
    }
    pthread_mutex_lock(&g_io);
    pthread_mutex_lock(&g_arb);
    void *h = g_chip; g_chip = NULL; g_owner = NULL; g_chip_name[0] = 0; g_chip_down = 0;
    pthread_mutex_unlock(&g_arb);
    pthread_mutex_unlock(&g_io);
    if (h) g_ops.close(h);
}

int voc_chip_present(void)
{
    pthread_mutex_lock(&g_arb);
    int p = chip_usable_locked();
    pthread_mutex_unlock(&g_arb);
    return p;
}

const char *voc_chip_name(void) { return g_chip_name; }

const char *voc_sw_version(void)
{
    static char v[48];
    if (!v[0]) snprintf(v, sizeof v, "mbelib-neo %s", mbe_versionString());
    return v;
}

unsigned voc_wire_bytes(voc_fmt_t fmt) { return fmt == VOC_FMT_YSF ? FRAMES_VOICE49_BYTES : FRAMES_AMBE72_BYTES; }

/* ---- streams ------------------------------------------------------------- */
voc_stream_t *voc_stream_create(voc_fmt_t fmt, int decode, const char *id)
{
    voc_stream_t *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    s->fmt = fmt;
    s->decode = decode;
    snprintf(s->id, sizeof s->id, "%s", id ? id : "?");
    swvoc_family_t fam = (fmt == VOC_FMT_DSTAR) ? SWVOC_AMBE2400 : SWVOC_AMBE2450;
    if (decode) { s->dec = swdec_create(fam); if (s->dec) swdec_set_gain(s->dec, g_cfg.sw_dec_gain); }
    else        { s->enc = swenc_create(fam); if (s->enc) swenc_set_gain(s->enc, g_cfg.sw_enc_gain); }
    if (!s->dec && !s->enc) { free(s); return NULL; }
    swvoc_window_reset(&s->win);
    return s;
}

void voc_stream_release(voc_stream_t *s)
{
    if (!s) return;
    pthread_mutex_lock(&g_arb);
    release_locked(s, "stream stopped");
    pthread_mutex_unlock(&g_arb);
}

void voc_stream_destroy(voc_stream_t *s)
{
    if (!s) return;
    voc_stream_release(s);
    /* Wait out an in-flight transaction of this stream before freeing it. */
    pthread_mutex_lock(&g_io);
    pthread_mutex_unlock(&g_io);
    swdec_destroy(s->dec);
    swenc_destroy(s->enc);
    free(s);
}

void voc_stream_reset(voc_stream_t *s)
{
    if (!s) return;
    voc_stream_release(s);
    swdec_reset(s->dec);
    swenc_reset(s->enc);
    swvoc_window_reset(&s->win);
}

/* ---- format plumbing ----------------------------------------------------- */

/* Software decode of one wire frame (NULL = erasure). */
static int sw_decode(voc_stream_t *s, const uint8_t *wire, int16_t pcm[160], swvoc_info_t *inf)
{
    if (!wire) return swdec_erasure(s->dec, pcm, inf);
    char fr[4][24], d[49];
    switch (s->fmt) {
    case VOC_FMT_DMR:
        frames_dmr_air_to_fr(wire, fr);
        return swdec_frame(s->dec, (const char (*)[24])fr, pcm, inf);
    case VOC_FMT_YSF:
        frames_voice49_to_d(wire, d);
        return swdec_params(s->dec, d, pcm, inf);
    case VOC_FMT_DSTAR:
    default:
        if (frames_dstar_to_fr(wire, fr) < 0) return -1;
        return swdec_frame(s->dec, (const char (*)[24])fr, pcm, inf);
    }
}

/* Software encode into a wire frame. */
static int sw_encode(voc_stream_t *s, const int16_t pcm[160], uint8_t *wire)
{
    char fr[4][24], d[49];
    switch (s->fmt) {
    case VOC_FMT_DMR:
        if (swenc_frame(s->enc, pcm, fr) < 0) return -1;
        frames_fr_to_dmr_air((const char (*)[24])fr, wire);
        return 0;
    case VOC_FMT_YSF:
        if (swenc_params(s->enc, pcm, d) < 0) return -1;
        frames_d_to_voice49(d, wire);
        return 0;
    case VOC_FMT_DSTAR:
    default:
        if (swenc_frame(s->enc, pcm, fr) < 0) return -1;
        return frames_fr_to_dstar((const char (*)[24])fr, wire);
    }
}

/* Wire frame -> the 9 bytes the chip takes at its rate. */
static int wire_to_chip(voc_stream_t *s, const uint8_t *wire, uint8_t chip9[9])
{
    uint8_t air[9];
    char d[49];
    switch (s->fmt) {
    case VOC_FMT_DMR:
        if (g_cfg.hw_canonical) frames_dmr_air_to_canonical(wire, chip9); else memcpy(chip9, wire, 9);
        return 0;
    case VOC_FMT_YSF:      /* re-protect the 49 bits into a 72-bit DMR frame */
        frames_voice49_to_d(wire, d);
        if (frames_d_to_dmr_air(d, air) < 0) return -1;
        if (g_cfg.hw_canonical) frames_dmr_air_to_canonical(air, chip9); else memcpy(chip9, air, 9);
        return 0;
    case VOC_FMT_DSTAR:
    default:
        memcpy(chip9, wire, 9);
        return 0;
    }
}

/* The chip's 9 bytes -> wire frame. */
static int chip_to_wire(voc_stream_t *s, const uint8_t chip9[9], uint8_t *wire)
{
    uint8_t air[9];
    char d[49];
    switch (s->fmt) {
    case VOC_FMT_DMR:
        if (g_cfg.hw_canonical) frames_dmr_canonical_to_air(chip9, wire); else memcpy(wire, chip9, 9);
        return 0;
    case VOC_FMT_YSF:      /* strip the AMBE FEC back to the 49 bits */
        if (g_cfg.hw_canonical) frames_dmr_canonical_to_air(chip9, air); else memcpy(air, chip9, 9);
        if (frames_dmr_air_to_d(air, d, NULL) < 0) return -1;
        frames_d_to_voice49(d, wire);
        return 0;
    case VOC_FMT_DSTAR:
    default:
        memcpy(wire, chip9, 9);
        return 0;
    }
}

/* ---- decode -------------------------------------------------------------- */
int voc_decode(voc_stream_t *s, const uint8_t *wire, int16_t pcm[160], voc_result_t *r)
{
    if (!s || !s->dec) return -1;

    /* 1. Software, every frame: it classifies the frame (the arbiter's
     *    evidence), keeps its state warm, and is the fallback. */
    int16_t sw[160];
    swvoc_info_t inf = { SWVOC_MUTED, -1 };
    int sw_ok = sw_decode(s, wire, sw, &inf) == 0;
    if (!sw_ok) { memset(sw, 0, sizeof sw); inf.cls = SWVOC_MUTED; inf.errors = -1; }
    swvoc_window_push(&s->win, &inf);
    int claim = swvoc_window_ok(&s->win, SWVOC_WIN_CLAIM);
    int hold  = swvoc_window_ok(&s->win, SWVOC_WIN_HOLD);
    int64_t now = now_ms();

    /* 2. Ownership. */
    pthread_mutex_lock(&g_arb);
    if (g_owner == s) {
        if (hold) g_owner_last = now;
        else if (now - g_owner_last > g_cfg.rx_hang_ms) release_locked(s, "transmission ended");
    } else if (claim && chip_usable_locked() && owner_idle_locked(now)) {
        acquire_locked(s, 0, now);
    }
    int mine = (g_owner == s);
    pthread_mutex_unlock(&g_arb);

    /* 3. The chip, if this stream holds it and there is a frame to give it
     *    (an erasure has none: that frame is concealed in software). */
    int16_t hw[160];
    int hw_ok = 0;
    if (mine && wire) {
        uint8_t chip9[9];
        if (wire_to_chip(s, wire, chip9) == 0) hw_ok = chip_xact(s, chip9, hw, NULL, NULL) == 0;
    }

    /* 4. Pick the output: warm-up plays software, then one crossfade frame,
     *    then the chip. */
    voc_backend_t be = VOC_BE_SW;
    if (hw_ok) {
        pthread_mutex_lock(&g_arb);
        int ph = (g_owner == s) ? g_phase : PH_LIVE;
        if (g_owner == s) {
            if (ph == PH_WARM && --g_warm_left <= 0) g_phase = PH_XFADE;
            else if (ph == PH_XFADE) g_phase = PH_LIVE;
        }
        pthread_mutex_unlock(&g_arb);
        if (ph == PH_LIVE) be = VOC_BE_CHIP;
        else if (ph == PH_XFADE) be = VOC_BE_XFADE;
    }
    if (be == VOC_BE_CHIP) {
        memcpy(pcm, hw, sizeof hw);
    } else if (be == VOC_BE_XFADE) {
        for (int i = 0; i < 160; i++) {
            float w = (i + 0.5f) / 160.0f;
            pcm[i] = (int16_t)lrintf((1.0f - w) * sw[i] + w * hw[i]);
        }
    } else {
        if (!sw_ok) return -1;
        memcpy(pcm, sw, sizeof sw);
    }
    if (r) { r->backend = be; r->errors = inf.errors; r->cls = (int)inf.cls; }
    return 0;
}

/* ---- encode -------------------------------------------------------------- */
static float frame_dbfs(const int16_t pcm[160])
{
    double acc = 0;
    for (int i = 0; i < 160; i++) acc += (double)pcm[i] * pcm[i];
    double ms = acc / 160.0;
    return ms <= 0 ? -200.0f : (float)(10.0 * log10(ms / (32768.0 * 32768.0)));
}

int voc_encode(voc_stream_t *s, const int16_t pcm[160], uint8_t *wire, voc_result_t *r)
{
    if (!s || !s->enc) return -1;

    /* Software encodes every frame, so its analysis state is continuous if
     * the chip goes away mid-over. */
    uint8_t sw_wire[9];
    int sw_ok = sw_encode(s, pcm, sw_wire) == 0;

    int voice = frame_dbfs(pcm) > g_cfg.tx_vad_dbfs;
    int64_t now = now_ms();

    pthread_mutex_lock(&g_arb);
    if (g_owner == s) {
        if (voice) g_owner_last = now;
        else if (now - g_owner_last > g_cfg.tx_hang_ms) release_locked(s, "quiet");
    } else if (voice && chip_usable_locked() &&
               (!g_owner || !g_owner_tx || owner_idle_locked(now))) {
        acquire_locked(s, 1, now);       /* transmit beats receive */
    }
    int mine = (g_owner == s);
    pthread_mutex_unlock(&g_arb);

    if (mine) {
        uint8_t chip9[9];
        if (chip_xact(s, NULL, NULL, pcm, chip9) == 0 && chip_to_wire(s, chip9, wire) == 0) {
            if (r) { r->backend = VOC_BE_CHIP; r->errors = -1; r->cls = 0; }
            return 0;
        }
    }
    if (!sw_ok) return -1;
    memcpy(wire, sw_wire, voc_wire_bytes(s->fmt));
    if (r) { r->backend = VOC_BE_SW; r->errors = -1; r->cls = 0; }
    return 0;
}
