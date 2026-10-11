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

#include "mdns.h"
#include "slc_vocoderd.h"
#include <stdio.h>
#ifdef HAVE_AVAHI
#include <avahi-client/client.h>
#include <avahi-client/publish.h>
#include <avahi-common/error.h>
#include <avahi-common/simple-watch.h>
#include <pthread.h>

static AvahiSimplePoll *poller;
static AvahiClient *client;
static AvahiEntryGroup *group;
static pthread_t tid;
static int started;   /* the poll thread exists (only then may it be joined) */

static void *poll_thread(void *x){(void)x; avahi_simple_poll_loop(poller); return NULL;}
static void client_cb(AvahiClient *c, AvahiClientState state, void *userdata)
{
    (void)userdata;
    if (state == AVAHI_CLIENT_FAILURE) avahi_simple_poll_quit(poller);
    client=c;
}
int mdns_start(const char *name,uint16_t port)
{
    int err=0;
    poller=avahi_simple_poll_new(); if(!poller) return -1;
    client=avahi_client_new(avahi_simple_poll_get(poller),0,client_cb,NULL,&err);
    if(!client){fprintf(stderr,"Avahi: %s\n",avahi_strerror(err)); return -1;}
    group=avahi_entry_group_new(client,NULL,NULL); if(!group) return -1;
    err=avahi_entry_group_add_service(group,AVAHI_IF_UNSPEC,AVAHI_PROTO_UNSPEC,0,
        name,"_slc-codec._tcp",NULL,NULL,port,
        "proto=sdr-slc","proto_ver=1.2","functions=codec","codecs=ambe",
        "model=sdr-slc-vocoderd (AMBE-3000 + mbelib-neo)","fw=" SLC_VERSION,NULL);
    if(err<0){fprintf(stderr,"Avahi add: %s\n",avahi_strerror(err)); return -1;}
    if(avahi_entry_group_commit(group)<0) return -1;
    if(pthread_create(&tid,NULL,poll_thread,NULL)!=0) return -1;
    started=1;
    return 0;
}
void mdns_stop(void)
{
    if(poller){avahi_simple_poll_quit(poller); if(started) pthread_join(tid,NULL);}
    started=0;
    if(group) avahi_entry_group_free(group);
    if(client) avahi_client_free(client);
    if(poller) avahi_simple_poll_free(poller);
    group=NULL; client=NULL; poller=NULL;
}
#else
int mdns_start(const char *name,uint16_t port){(void)name;(void)port;fprintf(stderr,"mDNS disabled: rebuild with libavahi-client-dev\n");return -1;}
void mdns_stop(void){}
#endif
