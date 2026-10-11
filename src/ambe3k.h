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

/* ambe3k.h - minimal C interface to a DVSI AMBE-3000 vocoder on a serial
 * port (DVMEGA DVstick30 / ThumbDV / NWDR AMBE3000 and compatibles).
 *
 * Scope: two AMBE rates, both a 72-bit / 9-byte FEC'd channel frame, one
 * 20 ms frame in each direction per call:
 *   - DMR   AMBE+2  3600 bps, 2450x1150  (the daemon's ambe-dmr / ambe-ysf path)
 *   - D-STAR AMBE   3600 bps, 2400x1200  (ambe-dstar)
 * The chip is a single-context vocoder configured for exactly one rate at a
 * time (PKT_RATEP); ambe3k_set_rate() switches it and, like every RATEP
 * write, re-initialises the vocoder state.
 *
 * This is a standalone module: it depends only on libc + Linux termios/sysfs,
 * no vocoder library, no json-c, and is validated on its own by
 * tools/ambe3k_loopback.c.
 *
 * Robustness (new in sdr-slc-vocoderd): every exchange flushes stale input
 * first, codec exchanges time out after 80 ms, the chip's acknowledgement
 * status is checked, and a vanished device is reported as AMBE3K_EGONE
 * rather than as a timeout, so the daemon can drop it at once and re-detect
 * it when it comes back.
 *
 * Frame/rate constants and byte order were taken from the DVSI AMBE-3000
 * manual and cross-checked against f4exb/serialDV (GPLv3), which is the
 * reference for the packet framing. This file is original code.
 */
#ifndef AMBE3K_H
#define AMBE3K_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMBE3K_PCM_SAMPLES  160   /* 20 ms @ 8 kHz                       */
#define AMBE3K_AMBE_BYTES   9     /* 72 coded bits: DMR 2450x1150 and D-STAR 2400x1200 alike */

/* Vocoder rate the chip is configured for. Both rates exchange the same
 * 9-byte channel frame over the packet interface; only the RATEP differs. */
typedef enum {
    AMBE3K_RATE_DMR = 0,   /* AMBE+2 3600/2450x1150: DMR, YSF, NXDN, dPMR */
    AMBE3K_RATE_DSTAR      /* AMBE   3600/2400x1200: D-STAR               */
} ambe3k_rate_t;
#define AMBE3K_PRODID_MAX   64

typedef struct ambe3k ambe3k_t;   /* opaque handle to one open device    */

/* Error returns of the session and codec calls (all negative). */
#define AMBE3K_ETIMEOUT (-1)  /* no (complete) reply in time; chip may recover */
#define AMBE3K_EGONE    (-2)  /* the device disappeared (unplugged, USB reset) */
#define AMBE3K_EPROTO   (-3)  /* wrong reply type, short reply, or rejected    */

/* ---- discovery -----------------------------------------------------------
 * Outcome of a scan. The daemon reacts differently to each: use the chip,
 * warn loudly and fall back, or fall back silently.                          */
typedef enum {
    AMBE3K_FOUND = 0,     /* a working AMBE-3000 was found (and, via
                             ambe3k_autodetect, opened + configured for DMR)  */
    AMBE3K_ABSENT,        /* no candidate USB-serial bridge present at all —
                             the normal "no dongle on this box" case          */
    AMBE3K_PERMISSION,    /* a candidate IS present but could not be opened
                             (permission denied — user not in the tty group)  */
    AMBE3K_NO_RESPONSE    /* a candidate opened but did not identify as an
                             AMBE-3000 (wrong device, cabling, or port busy)   */
} ambe3k_status_t;

/* Scan every /dev/ttyUSB*, keep the USB bridge chips the DVstick ships with,
 * and probe each with the product-ID handshake. Classifies the result so the
 * caller can tell "no dongle" apart from "dongle I can't open".
 *
 * out_path    the device involved (set for FOUND/PERMISSION/NO_RESPONSE).
 * out_prodid  the chip id, e.g. "AMBE3000F" (FOUND only; may be NULL).
 * out_hint    if non-NULL, receives a one-line human-readable remedy for the
 *             non-FOUND cases (e.g. which group to join); empty otherwise.
 * Prints nothing itself — the caller logs however it likes.                  */
ambe3k_status_t ambe3k_find(char *out_path, size_t out_len,
                            char *out_prodid, size_t prodid_len,
                            char *out_hint, size_t hint_len);

/* List the candidate USB-serial bridges currently present (the same filter
 * ambe3k_find applies), without opening any of them. Writes their tty names,
 * sorted and space-separated, into buf (e.g. "ttyUSB0 ttyUSB2") and returns
 * how many there are. The daemon re-probes only when this signature changes,
 * so an unrelated serial device is not poked every few seconds. */
int ambe3k_candidates(char *buf, size_t len);

/* Daemon-startup convenience: ambe3k_find(), and on AMBE3K_FOUND also open +
 * configure the device. On AMBE3K_FOUND *out_handle is the open device (caller
 * owns it; close with ambe3k_close). On every other status *out_handle is NULL
 * and the caller should fall back to its software codec. Same out_* semantics
 * as ambe3k_find.                                                             */
ambe3k_status_t ambe3k_autodetect(ambe3k_t **out_handle,
                                  char *out_path, size_t out_len,
                                  char *out_prodid, size_t prodid_len,
                                  char *out_hint, size_t hint_len);

/* Low-level probe of one specific device: open at 460800, send a product-ID
 * request, and return 1 iff the reply identifies an AMBE-3000 (its id string
 * is copied to prodid, up to n bytes). Returns 0 if it answered but is not an
 * AMBE-3000, -1 on open/IO/timeout error. Sends nothing but the benign 5-byte
 * product-ID request and never reconfigures the device. Used by ambe3k_find;
 * exposed for diagnostics.                                                    */
int ambe3k_probe(const char *devpath, char *prodid, size_t n);

/* ---- session -------------------------------------------------------------
 * Open the device, drop the FTDI latency timer, verify it is an AMBE-3000,
 * and configure it for the DMR rate. Returns NULL on any failure (a message
 * is printed to stderr). Close with ambe3k_close().                          */
ambe3k_t *ambe3k_open(const char *devpath);
void      ambe3k_close(ambe3k_t *h);

/* The product-id string read during open (e.g. "AMBE3000F"). */
const char *ambe3k_prodid(const ambe3k_t *h);

/* The device path this handle was opened on. */
const char *ambe3k_path(const ambe3k_t *h);

/* Set the chip's speech gains in dB (each clamped to [-90, 90]).
 *   in_db   amplifies (>0) / attenuates (<0) PCM BEFORE encoding.
 *   out_db  amplifies (>0) / attenuates (<0) PCM AFTER decoding — use this to
 *           raise the decoded audio level to match another backend.
 * The gains persist in the chip until changed. 0, or a negative AMBE3K_E* code. */
int ambe3k_set_gain(ambe3k_t *h, int in_db, int out_db);

/* Configure the chip for `rate` (sends the matching PKT_RATEP and waits for
 * the acknowledgement). Also resets the vocoder state, so a stream that takes
 * over the chip at a different rate starts clean. Idempotent in effect but
 * always performs the round-trip. 0, or a negative AMBE3K_E* code. */
int ambe3k_set_rate(ambe3k_t *h, ambe3k_rate_t rate);

/* The rate the chip is currently configured for (as last accepted). */
ambe3k_rate_t ambe3k_rate(const ambe3k_t *h);

/* Human-readable name of a rate, e.g. "DMR 3600/2450x1150". */
const char *ambe3k_rate_name(ambe3k_rate_t rate);

/* Re-apply the CURRENT rate configuration to return the vocoder to a known
 * state. Used by the codec arbiter at talk-spurt boundaries so a new
 * utterance does not start on the tail of the previous holder's decoder/
 * encoder state. This re-sends the (already validated) RATEP packet rather
 * than a bespoke soft-reset field, so it introduces no untested control
 * packet; if a dedicated PKT_INIT is ever verified it can replace the body.
 * Costs one control round-trip. 0, or a negative AMBE3K_E* code. */
int ambe3k_reset(ambe3k_t *h);

/* ---- codec ---------------------------------------------------------------
 * One 20 ms frame each way. PCM is HOST-order int16 (this module handles the
 * big-endian conversion to/from the chip internally). ambe9 is the raw 9-byte
 * DMR channel frame, passed through verbatim in both directions — no bit
 * reordering happens here. Return 0, or a negative AMBE3K_E* code. */
int ambe3k_encode(ambe3k_t *h, const int16_t pcm[AMBE3K_PCM_SAMPLES],
                  uint8_t ambe9[AMBE3K_AMBE_BYTES]);
int ambe3k_decode(ambe3k_t *h, const uint8_t ambe9[AMBE3K_AMBE_BYTES],
                  int16_t pcm[AMBE3K_PCM_SAMPLES]);

#ifdef __cplusplus
}
#endif
#endif /* AMBE3K_H */
