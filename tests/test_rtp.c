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
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(void){uint8_t payload[7]={1,2,3,4,5,6,0x80},pkt[64];size_t n=rtp_build(pkt,sizeof(pkt),1,97,123,456,789,payload,7);assert(n==19);struct rtp_view v;assert(rtp_parse(pkt,n,&v)==0);assert(v.marker&&v.payload_type==97&&v.sequence==123&&v.timestamp==456&&v.ssrc==789&&v.payload_len==7&&!memcmp(v.payload,payload,7));puts("RTP test OK");return 0;}
