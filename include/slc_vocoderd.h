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

#ifndef SLC_VOCODERD_H
#define SLC_VOCODERD_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <netinet/in.h>
#include "vocoder.h"

#define SLC_VERSION "1.0.0"
#define SLC_DEFAULT_PORT 4630
#define SLC_MAX_JSON 65536

/* Wire formats (SDR-SLC-RTP-1.0); see src/frames.h for the exact bit layouts.
 *   ambe-dmr    72-bit AMBE+2 channel frame, DMR on-air order, 9 bytes
 *   ambe-ysf    49-bit AMBE+2 voice bits, no AMBE FEC (YSF V/D mode 2), 7 bytes
 *   ambe-dstar  72-bit D-STAR DV voice data, as on air, 9 bytes */
#define SLC_DMR_FRAME_BYTES     9
#define SLC_DMR_FRAME_BITS      72
#define SLC_VOICE49_FRAME_BYTES 7
#define SLC_VOICE49_FRAME_BITS  49
#define SLC_DSTAR_FRAME_BYTES   9
#define SLC_DSTAR_FRAME_BITS    72
#define SLC_PCM_BYTES           320

struct slc_stream {
    bool open;
    bool active;
    bool decode;          /* true: AMBE->PCM (receive), false: PCM->AMBE (transmit) */
    voc_fmt_t fmt;        /* which AMBE wire format                                  */
    char id[32];
    char from[32];
    char to[32];
    int in_fd;
    int out_fd;
    struct sockaddr_storage output_addr;
    socklen_t output_addr_len;
    uint8_t input_pt;
    uint8_t output_pt;
    unsigned input_fpp;
    unsigned output_fpp;
    uint16_t out_seq;
    uint32_t out_ts;
    uint32_t out_ssrc;
    pthread_t thread;
    pthread_mutex_t lock;
    int control_fd;
    char peer[64];               /* control connection's peer, for diagnostics   */
    voc_stream_t *voc;           /* chip arbitration + mbelib-neo state           */
    int64_t opened_ms;           /* when open_stream created it                   */
    _Atomic int64_t last_frame_ms; /* arrival of the last media frame (0 = none)  */
};

/* Concurrent conversions. Every stream carries its own mbelib-neo instance,
 * which is cheap (a few kB and well under a millisecond of CPU per frame), so
 * the limit is generous: an armed transmit plus a receive slice per timeslot
 * and mode, for more than one client, fits easily. sdr-slc-codecd's 4 was
 * the cause of the client's "busy" errors. `capacity` in the capabilities
 * advertises this number. Allocation and lookup happen only inside the
 * command handler, which is serialised behind handle_lock. */
#define SLC_MAX_STREAMS 16

struct slc_server {
    int listen_fd;
    uint16_t port;
    char name[128];
    struct slc_stream streams[SLC_MAX_STREAMS];
};

#endif
