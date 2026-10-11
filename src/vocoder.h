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

/* vocoder.h — the vocoder service core: one AMBE-3000 shared by all streams,
 * with mbelib-neo underneath every stream as the per-frame fallback.
 *
 * ---- the policy -----------------------------------------------------------
 * The AMBE-3000 is the preferred vocoder for every format, both directions.
 * It has a single stateful vocoder context, so it serves one stream at a
 * time; every other stream (and any stream whenever the chip is absent,
 * unplugged or failing) is served by its own mbelib-neo instance, frame by
 * frame, with no gap. The question is only WHO gets the chip, and the
 * answer is: whoever is actually carrying voice.
 *
 *   Transmit (PCM -> AMBE) claims the chip when its audio rises above the
 *     voice threshold (--tx-vad-dbfs) and keeps it until it has been quiet
 *     for --tx-hang-ms. An "armed" transmit stream fed with silence between
 *     overs therefore never holds the chip. Transmit beats receive: a
 *     receive owner drops to its software decoder at once.
 *
 *   Receive (AMBE -> PCM) claims the chip when its frames show a real
 *     transmission (see SWVOC_WIN_CLAIM in swvoc.h: a 120 ms window of clean
 *     voice frames that random noise passes about once every three hours),
 *     at ANY point in an over, not only at its start. It keeps the chip while
 *     the looser hold test passes, and releases it --rx-hang-ms after the
 *     last evidence. The first receiver to claim keeps it for its over.
 *
 *   Mid-over takeover without a click. A stream that claims the chip resets
 *     it (which is what a new utterance needs anyway), but a freshly reset
 *     AMBE+2 decoder needs a few frames before its gain and spectral
 *     prediction match the talker. So a receiver first runs the chip
 *     silently for --warmup-frames frames while its software decoder keeps
 *     playing, then crossfades software -> chip over one 20 ms frame. Both
 *     decoders produce DVSI's reference level, so the seam is inaudible.
 *     Losing the chip (transmit preemption, chip failure) switches back to
 *     the software decoder, which has decoded every frame all along.
 *
 * ---- the chip's life ------------------------------------------------------
 * A background thread owns discovery. If no chip is present at startup it
 * keeps looking (re-probing only when the set of USB serial bridges changes,
 * so unrelated serial devices are not poked); if the chip disappears or
 * fails repeatedly it is closed and re-detected when it returns. Chip I/O
 * never blocks a stream that does not own the chip. */
#ifndef SLC_VOCODER_H
#define SLC_VOCODER_H
#include <stdint.h>

typedef enum { VOC_FMT_DMR = 0, VOC_FMT_YSF, VOC_FMT_DSTAR } voc_fmt_t;

/* Which backend produced the emitted frame. */
typedef enum {
    VOC_BE_SW = 0,     /* mbelib-neo                                          */
    VOC_BE_CHIP,       /* AMBE-3000                                           */
    VOC_BE_XFADE       /* the one crossfade frame from software to the chip   */
} voc_backend_t;

struct voc_config {
    int         disable_hw;      /* --no-hw: never use an AMBE-3000          */
    const char *hw_dev;          /* --hw-dev PATH, or NULL to scan USB       */
    int         hw_canonical;    /* --hw-canonical: chip uses DVSI canonical DMR order */
    int         hw_in_gain_db;   /* AMBE-3000 encode input gain              */
    int         hw_out_gain_db;  /* AMBE-3000 decode output gain             */
    float       sw_dec_gain;     /* linear gain on mbelib-neo decoded PCM    */
    float       sw_enc_gain;     /* linear gain on PCM into mbelib-neo       */
    float       tx_vad_dbfs;     /* transmit voice threshold, frame RMS dBFS */
    int         tx_hang_ms;      /* transmit keeps the chip this long after voice */
    int         rx_hang_ms;      /* receive keeps the chip this long after evidence */
    int         warmup_frames;   /* silent chip frames before a receive crossfade */
    int         verbose;         /* log ownership changes when > 0           */
};
void voc_config_defaults(struct voc_config *c);

/* Start the service core (and the chip discovery thread). 0 or -1. */
int  voc_init(const struct voc_config *c);
void voc_shutdown(void);

int         voc_chip_present(void);     /* an AMBE-3000 is open and healthy now */
const char *voc_chip_name(void);        /* e.g. "AMBE3000F on /dev/ttyUSB0", or "" */
const char *voc_sw_version(void);       /* e.g. "mbelib-neo 2.4.0"            */

typedef struct voc_stream voc_stream_t;

/* One conversion. `id` is only used in log lines. */
voc_stream_t *voc_stream_create(voc_fmt_t fmt, int decode, const char *id);
void          voc_stream_destroy(voc_stream_t *s);     /* also releases the chip */
void          voc_stream_reset(voc_stream_t *s);       /* new session (start_codec) */
void          voc_stream_release(voc_stream_t *s);     /* drop any chip claim (stop_codec) */

typedef struct {
    voc_backend_t backend;
    int errors;          /* corrected bit errors on decode (software FEC), -1 if unknown */
    int cls;             /* swvoc_class_t of the frame (decode only)           */
} voc_result_t;

/* Decode one wire frame (9 bytes, or 7 for YSF) into 20 ms of PCM. wire ==
 * NULL is an RTP erasure (concealed). 0, or -1 if no backend could decode. */
int voc_decode(voc_stream_t *s, const uint8_t *wire, int16_t pcm[160], voc_result_t *r);

/* Encode 20 ms of PCM into one wire frame. 0 or -1. */
int voc_encode(voc_stream_t *s, const int16_t pcm[160], uint8_t *wire, voc_result_t *r);

/* Wire bytes of one frame of the format. */
unsigned voc_wire_bytes(voc_fmt_t fmt);

/* ---- test hook -------------------------------------------------------------
 * The chip I/O is reached through this table so the arbiter can be tested
 * against a simulated chip. The daemon uses the AMBE-3000 driver; tests may
 * install their own before voc_init(). */
struct voc_chip_ops {
    void *(*open_any)(char *name, unsigned long name_len);   /* discover + open, NULL if none */
    void  (*close)(void *h);
    int   (*set_rate)(void *h, int dstar);
    int   (*decode)(void *h, const uint8_t in9[9], int16_t pcm[160]);
    int   (*encode)(void *h, const int16_t pcm[160], uint8_t out9[9]);
    int   (*changed)(void);   /* 1 if it is worth probing again (device set changed) */
};
void voc_set_chip_ops(const struct voc_chip_ops *ops);

#endif
