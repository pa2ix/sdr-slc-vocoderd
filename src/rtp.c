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

#include "rtp.h"
#include <arpa/inet.h>
#include <string.h>

static uint16_t rd16(const uint8_t *p) { uint16_t v; memcpy(&v,p,2); return ntohs(v); }
static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v,p,4); return ntohl(v); }
static void wr16(uint8_t *p,uint16_t v){v=htons(v);memcpy(p,&v,2);} 
static void wr32(uint8_t *p,uint32_t v){v=htonl(v);memcpy(p,&v,4);} 

int rtp_parse(const uint8_t *p, size_t len, struct rtp_view *o)
{
    if (!p || !o || len < 12) return -1;
    unsigned v = p[0] >> 6, padding=(p[0]>>5)&1, ext=(p[0]>>4)&1, cc=p[0]&0x0f;
    if (v != 2) return -1;
    size_t off = 12 + 4U*cc;
    if (off > len) return -1;
    if (ext) {
        if (off + 4 > len) return -1;
        uint16_t words = rd16(p + off + 2);
        off += 4 + 4U*words;
        if (off > len) return -1;
    }
    size_t end = len;
    if (padding) {
        uint8_t n = p[len-1];
        if (n == 0 || n > end-off) return -1;
        end -= n;
    }
    o->marker = (p[1]&0x80)!=0;
    o->payload_type = p[1]&0x7f;
    o->sequence = rd16(p+2);
    o->timestamp = rd32(p+4);
    o->ssrc = rd32(p+8);
    o->payload = p+off;
    o->payload_len = end-off;
    return 0;
}

size_t rtp_build(uint8_t *o,size_t cap,bool marker,uint8_t pt,uint16_t seq,uint32_t ts,uint32_t ssrc,const uint8_t *payload,size_t plen)
{
    if (!o || cap < 12+plen) return 0;
    o[0]=0x80; o[1]=(marker?0x80:0)|(pt&0x7f);
    wr16(o+2,seq); wr32(o+4,ts); wr32(o+8,ssrc);
    if (plen && payload) memcpy(o+12,payload,plen);
    return 12+plen;
}
