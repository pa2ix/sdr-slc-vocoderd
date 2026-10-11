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

#ifndef SLC_RTP_H
#define SLC_RTP_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

struct rtp_view {
    bool marker;
    uint8_t payload_type;
    uint16_t sequence;
    uint32_t timestamp;
    uint32_t ssrc;
    const uint8_t *payload;
    size_t payload_len;
};

int rtp_parse(const uint8_t *packet, size_t len, struct rtp_view *out);
size_t rtp_build(uint8_t *out, size_t cap, bool marker, uint8_t pt,
                 uint16_t sequence, uint32_t timestamp, uint32_t ssrc,
                 const uint8_t *payload, size_t payload_len);
#endif
