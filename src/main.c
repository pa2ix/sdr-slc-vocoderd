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

/* main.c — the SDR-SLC control plane (SDR-SLC-CP-1.0 codec block) and the
 * RTP media threads. The protocol side is unchanged from sdr-slc-codecd 1.0,
 * so existing clients work as before; all vocoder decisions live in
 * vocoder.c. */
#define _GNU_SOURCE                 /* TCP_KEEPIDLE / TCP_KEEPINTVL / TCP_KEEPCNT */
#include "slc_vocoderd.h"
#include "rtp.h"
#include "mdns.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <json-c/json.h>
#include <math.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* Verbosity: 0 = quiet (errors, backend changes, the low-rate RTP report),
 * 1 = -v control plane and chip ownership, 2 = -vv every frame. */
static int slc_verbose = 0;
#define LOGV(lvl, ...) do { if (slc_verbose >= (lvl)) { fprintf(stderr, "[V%d] ", (lvl)); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

static volatile sig_atomic_t quitting = 0;
static void on_signal(int s) { (void)s; quitting = 1; }

static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static uint32_t rnd32(void)
{
    uint32_t v = 0;
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd >= 0) { if (read(fd, &v, 4) != 4) v = 0; close(fd); }
    if (!v) v = (uint32_t)time(NULL) ^ (uint32_t)getpid();
    return v;
}

/* ---- control-plane framing: 4-byte big-endian length + JSON ------------- */
static int send_all(int fd, const void *buf, size_t n)
{
    const uint8_t *p = buf;
    while (n) {
        ssize_t r = send(fd, p, n, MSG_NOSIGNAL);
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        p += r; n -= (size_t)r;
    }
    return 0;
}

/* 1 = got n bytes, 0 = orderly close, -1 = error. */
static int recv_all(int fd, void *buf, size_t n)
{
    uint8_t *p = buf;
    while (n) {
        ssize_t r = recv(fd, p, n, 0);
        if (r == 0) return 0;
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        p += r; n -= (size_t)r;
    }
    return 1;
}

static int send_json(int fd, json_object *o)
{
    const char *s = json_object_to_json_string_ext(o, JSON_C_TO_STRING_PLAIN);
    uint32_t n = (uint32_t)strlen(s), be = htonl(n);
    return (send_all(fd, &be, 4) || send_all(fd, s, n)) ? -1 : 0;
}

static json_object *jstr(const char *s) { return json_object_new_string(s ? s : ""); }

static json_object *resp_ok(const char *id, json_object *result)
{
    json_object *o = json_object_new_object();
    json_object_object_add(o, "type", jstr("response"));
    json_object_object_add(o, "id", jstr(id));
    json_object_object_add(o, "status", jstr("ok"));
    json_object_object_add(o, "result", result ? result : json_object_new_object());
    return o;
}

static json_object *resp_err(const char *id, const char *code, const char *msg)
{
    json_object *o = json_object_new_object(), *e = json_object_new_object();
    json_object_object_add(o, "type", jstr("response"));
    json_object_object_add(o, "id", jstr(id));
    json_object_object_add(o, "status", jstr("error"));
    json_object_object_add(e, "code", jstr(code));
    json_object_object_add(e, "message", jstr(msg));
    json_object_object_add(o, "error", e);
    return o;
}

static void send_state(int fd, const char *sid, bool active)
{
    json_object *o = json_object_new_object(), *p = json_object_new_object();
    json_object_object_add(o, "type", jstr("event"));
    json_object_object_add(o, "event", jstr("stream_state"));
    json_object_object_add(p, "stream_id", jstr(sid));
    json_object_object_add(p, "streaming", json_object_new_boolean(active));
    json_object_object_add(p, "function", jstr("codec"));
    json_object_object_add(o, "params", p);
    send_json(fd, o);
    json_object_put(o);
}

/* ---- capabilities -------------------------------------------------------- */
static json_object *frame_format(const char *id, const char *mode, int rate_bps, int bits, int bytes)
{
    json_object *f = json_object_new_object();
    json_object_object_add(f, "id", jstr(id));
    json_object_object_add(f, "kind", jstr("frame"));
    json_object_object_add(f, "codec", jstr("ambe"));
    json_object_object_add(f, "mode", jstr(mode));
    json_object_object_add(f, "rate_bps", json_object_new_int(rate_bps));
    json_object_object_add(f, "frame_ms", json_object_new_int(20));
    json_object_object_add(f, "frame_bits", json_object_new_int(bits));
    json_object_object_add(f, "frame_bytes", json_object_new_int(bytes));
    return f;
}

static void add_conversion(json_object *conv, const char *from, const char *to)
{
    json_object *c = json_object_new_object();
    json_object_object_add(c, "from", jstr(from));
    json_object_object_add(c, "to", jstr(to));
    json_object_array_add(conv, c);
}

static json_object *capabilities(void)
{
    json_object *root = json_object_new_object(), *caps = json_object_new_object(), *codec = json_object_new_object();

    json_object *trans = json_object_new_array(), *t = json_object_new_object();
    json_object_object_add(t, "protocol", jstr("rtp"));
    json_object_object_add(t, "protocol_version", jstr("1.0"));
    json_object_array_add(trans, t);

    json_object *fmts = json_object_new_array();
    json_object *pcm = json_object_new_object();
    json_object_object_add(pcm, "id", jstr("pcm8k"));
    json_object_object_add(pcm, "kind", jstr("pcm"));
    json_object_object_add(pcm, "sample_rate_sps", json_object_new_int(8000));
    json_object_object_add(pcm, "format", jstr("s16be"));
    json_object_object_add(pcm, "channels", json_object_new_int(1));
    json_object_array_add(fmts, pcm);
    json_object_array_add(fmts, frame_format("ambe-dmr", "dmr", 3600, SLC_DMR_FRAME_BITS, SLC_DMR_FRAME_BYTES));
    /* rate_bps 2450 x 20 ms = 49 bits, so the client's size check passes. */
    json_object_array_add(fmts, frame_format("ambe-ysf", "ysf", 2450, SLC_VOICE49_FRAME_BITS, SLC_VOICE49_FRAME_BYTES));
    /* D-STAR is now always offered: mbelib-neo has an AMBE 3600x2400 decoder
     * and encoder, so it no longer depends on an AMBE-3000 being present. */
    json_object_array_add(fmts, frame_format("ambe-dstar", "dstar", 3600, SLC_DSTAR_FRAME_BITS, SLC_DSTAR_FRAME_BYTES));

    json_object *conv = json_object_new_array();
    add_conversion(conv, "ambe-dmr", "pcm8k");
    add_conversion(conv, "pcm8k", "ambe-dmr");
    add_conversion(conv, "ambe-ysf", "pcm8k");
    add_conversion(conv, "pcm8k", "ambe-ysf");
    add_conversion(conv, "ambe-dstar", "pcm8k");
    add_conversion(conv, "pcm8k", "ambe-dstar");

    json_object_object_add(codec, "tier", json_object_new_int(1));
    json_object_object_add(codec, "access", jstr("shared"));
    json_object_object_add(codec, "capacity", json_object_new_int(SLC_MAX_STREAMS));
    json_object_object_add(codec, "transports", trans);
    json_object_object_add(codec, "formats", fmts);
    json_object_object_add(codec, "conversions", conv);

    char version[256];
    int chip = voc_chip_present();
    if (chip) snprintf(version, sizeof version, "%s + %s", voc_chip_name(), voc_sw_version());
    else      snprintf(version, sizeof version, "%s", voc_sw_version());
    json_object *impl = json_object_new_object();
    json_object_object_add(impl, "backend", jstr(chip ? "AMBE3000" : "mbelib-neo"));
    json_object_object_add(impl, "backend_version", jstr(version));
    json_object_object_add(impl, "real_ambe", json_object_new_boolean(1));
    json_object_object_add(codec, "implementation", impl);

    json_object_object_add(caps, "schema", jstr("sdrslc.capabilities"));
    json_object_object_add(caps, "schema_version", json_object_new_int(2));
    json_object_object_add(caps, "codec", codec);
    json_object_object_add(root, "capabilities", caps);
    return root;
}

/* ---- sockets ------------------------------------------------------------- */
static int bind_udp(uint16_t *port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) { close(fd); return -1; }
    socklen_t n = sizeof a;
    getsockname(fd, (struct sockaddr *)&a, &n);
    *port = ntohs(a.sin_port);
    return fd;
}

static int resolve_dest(const char *ip, int port, struct sockaddr_storage *ss, socklen_t *sl)
{
    char service[16];
    snprintf(service, sizeof service, "%d", port);
    struct addrinfo hints = {0}, *ai = NULL;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_family = AF_UNSPEC;
    if (getaddrinfo(ip, service, &hints, &ai) != 0) return -1;
    memcpy(ss, ai->ai_addr, ai->ai_addrlen);
    *sl = ai->ai_addrlen;
    freeaddrinfo(ai);
    return 0;
}

static void local_ip_for(const struct sockaddr_storage *dst, char out[INET6_ADDRSTRLEN])
{
    strcpy(out, "127.0.0.1");
    int fd = socket(dst->ss_family, SOCK_DGRAM, 0);
    if (fd < 0) return;
    socklen_t dl = dst->ss_family == AF_INET ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6);
    if (connect(fd, (const struct sockaddr *)dst, dl) == 0) {
        struct sockaddr_storage s;
        socklen_t n = sizeof s;
        if (getsockname(fd, (struct sockaddr *)&s, &n) == 0) {
            const void *p = s.ss_family == AF_INET ? (const void *)&((struct sockaddr_in *)&s)->sin_addr
                                                   : (const void *)&((struct sockaddr_in6 *)&s)->sin6_addr;
            inet_ntop(s.ss_family, p, out, INET6_ADDRSTRLEN);
        }
    }
    close(fd);
}

static void addr_str(const struct sockaddr_storage *ss, char *ip, size_t iplen, int *port)
{
    snprintf(ip, iplen, "?");
    *port = 0;
    if (ss->ss_family == AF_INET) {
        const struct sockaddr_in *a = (const struct sockaddr_in *)ss;
        inet_ntop(AF_INET, &a->sin_addr, ip, (socklen_t)iplen);
        *port = ntohs(a->sin_port);
    } else if (ss->ss_family == AF_INET6) {
        const struct sockaddr_in6 *a = (const struct sockaddr_in6 *)ss;
        inet_ntop(AF_INET6, &a->sin6_addr, ip, (socklen_t)iplen);
        *port = ntohs(a->sin6_port);
    }
}

/* Detect a control connection whose client has silently vanished (Wi-Fi
 * gone, laptop asleep, a crash on a NAT'd network that never sends a FIN).
 * Without this, recv() waits forever and the client's streams keep their
 * slots: with sdr-slc-codecd's 4 slots, two such reconnects meant "busy".
 * Probe after 10 s idle, every 5 s, give up after 3 misses (~25 s), and
 * time out unacknowledged data after 30 s. */
static void set_keepalive(int fd)
{
    int on = 1, idle = 10, intvl = 5, cnt = 3;
    unsigned int user_timeout = 30000;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof on);
#ifdef TCP_KEEPIDLE
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof idle);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof intvl);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof cnt);
#endif
#ifdef TCP_USER_TIMEOUT
    setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &user_timeout, sizeof user_timeout);
#endif
    (void)idle; (void)intvl; (void)cnt; (void)user_timeout;
}

static void pcm_to_be(const int16_t *pcm, uint8_t out[320])
{
    for (int i = 0; i < 160; i++) { uint16_t v = (uint16_t)pcm[i]; out[2 * i] = (uint8_t)(v >> 8); out[2 * i + 1] = (uint8_t)v; }
}
static void pcm_from_be(const uint8_t in[320], int16_t *pcm)
{
    for (int i = 0; i < 160; i++) pcm[i] = (int16_t)(((uint16_t)in[2 * i] << 8) | in[2 * i + 1]);
}

/* ---- RTP media ----------------------------------------------------------- *
 * Counters answer "sent N, received 0" style questions on the client: is the
 * daemon receiving, discarding on payload type or size, or failing to send
 * back? A line on the first packet and every 100 after (~2 lines/s). */
struct rtp_stats {
    unsigned long rx, bad_parse, bad_pt, bad_len, erasure, tx, tx_fail, codec_fail, codec_ok, chip, sw;
    int reported_fail, last_be;
};

static void rtp_report(const struct slc_stream *s, const struct rtp_stats *st)
{
    char ip[INET6_ADDRSTRLEN];
    int port;
    addr_str(&s->output_addr, ip, sizeof ip, &port);
    fprintf(stderr, "[RTP] %s rx=%lu (bad_parse=%lu wrong_pt=%lu bad_len=%lu erasure=%lu) "
                    "codec(ok=%lu fail=%lu chip=%lu sw=%lu) tx=%lu to %s:%d fail=%lu\n",
            s->id, st->rx, st->bad_parse, st->bad_pt, st->bad_len, st->erasure,
            st->codec_ok, st->codec_fail, st->chip, st->sw, st->tx, ip, port, st->tx_fail);
}

static void rtp_send(struct slc_stream *s, struct rtp_stats *st, const uint8_t *pkt, size_t m)
{
    if (!m) return;
    if (sendto(s->out_fd, pkt, m, 0, (struct sockaddr *)&s->output_addr, s->output_addr_len) < 0) {
        st->tx_fail++;
        if (!st->reported_fail) {
            st->reported_fail = 1;
            fprintf(stderr, "[RTP] %s sendto failed: %s — the reply address the client gave is unreachable from this host\n",
                    s->id, strerror(errno));
        }
    } else {
        st->tx++;
    }
}

/* Count which backend produced the frame and announce each change. The line
 * is printed without -v because "which vocoder is in use" is exactly what an
 * operator watches; it changes at most a few times per over. */
static void backend_used(struct slc_stream *s, struct rtp_stats *st, voc_backend_t be)
{
    int chip = (be != VOC_BE_SW);
    if (chip) st->chip++; else st->sw++;
    if (st->last_be != chip) {
        st->last_be = chip;
        fprintf(stderr, "[codec] %s %s -> %s\n", s->id, s->decode ? "RX(decode)" : "TX(encode)",
                chip ? (s->fmt == VOC_FMT_DSTAR ? "AMBE-3000 chip (D-STAR rate)" : "AMBE-3000 chip") : voc_sw_version());
    }
}

static void *media_thread(void *arg)
{
    struct slc_stream *s = arg;
    uint8_t in[2048], out[1600];
    struct rtp_stats st = {0};
    st.last_be = -1;
    const unsigned ab = voc_wire_bytes(s->fmt);

    for (;;) {
        pthread_mutex_lock(&s->lock);
        bool run = s->active;
        pthread_mutex_unlock(&s->lock);
        if (!run) break;

        ssize_t n = recv(s->in_fd, in, sizeof in, 0);
        if (n < 0) { if (errno == EINTR) continue; break; }
        st.rx++;
        if (st.rx == 1 || st.rx % 100 == 0 || slc_verbose >= 2) rtp_report(s, &st);

        struct rtp_view rv;
        if (rtp_parse(in, (size_t)n, &rv) < 0) { st.bad_parse++; continue; }
        if (rv.payload_type != s->input_pt) { st.bad_pt++; continue; }
        s->last_frame_ms = now_ms();

        if (rv.payload_len == 0) {
            /* Erasure (SDR-SLC-RTP-1.0): the client knows this frame is bad.
             * Decode: conceal it (repeat, then mute after three). Encode: pass
             * the erasure on. */
            st.erasure++;
            if (s->decode) {
                int16_t pcm[160]; uint8_t pbe[320]; voc_result_t r;
                if (voc_decode(s->voc, NULL, pcm, &r) < 0) memset(pcm, 0, sizeof pcm);
                pcm_to_be(pcm, pbe);
                size_t m = rtp_build(out, sizeof out, rv.marker, s->output_pt, s->out_seq++, s->out_ts, s->out_ssrc, pbe, 320);
                s->out_ts += 160;
                rtp_send(s, &st, out, m);
            } else {
                size_t m = rtp_build(out, sizeof out, rv.marker, s->output_pt, s->out_seq++, s->out_ts, s->out_ssrc, NULL, 0);
                s->out_ts += 160;
                rtp_send(s, &st, out, m);
            }
            continue;
        }

        if (s->decode) {
            if (rv.payload_len % ab) { st.bad_len++; continue; }
            size_t frames = rv.payload_len / ab;
            for (size_t f = 0; f < frames; f++) {
                int16_t pcm[160]; uint8_t pbe[320]; voc_result_t r;
                if (voc_decode(s->voc, rv.payload + ab * f, pcm, &r) < 0) {
                    st.codec_fail++;
                    LOGV(2, "%s decode frame %zu FAILED", s->id, f);
                    continue;
                }
                st.codec_ok++;
                backend_used(s, &st, r.backend);
                LOGV(2, "%s decode frame %zu ok (%d bit errs, %s)", s->id, f, r.errors,
                     r.backend == VOC_BE_CHIP ? "chip" : r.backend == VOC_BE_XFADE ? "crossfade" : "sw");
                pcm_to_be(pcm, pbe);
                size_t m = rtp_build(out, sizeof out, rv.marker && f == 0, s->output_pt, s->out_seq++, s->out_ts, s->out_ssrc, pbe, 320);
                s->out_ts += 160;
                rtp_send(s, &st, out, m);
            }
        } else {
            if (rv.payload_len % SLC_PCM_BYTES) { st.bad_len++; continue; }
            size_t frames = rv.payload_len / SLC_PCM_BYTES;
            for (size_t f = 0; f < frames; f++) {
                int16_t pcm[160]; uint8_t ambe[9]; voc_result_t r;
                pcm_from_be(rv.payload + SLC_PCM_BYTES * f, pcm);
                if (voc_encode(s->voc, pcm, ambe, &r) < 0) {
                    st.codec_fail++;
                    LOGV(2, "%s encode frame %zu FAILED", s->id, f);
                    continue;
                }
                st.codec_ok++;
                backend_used(s, &st, r.backend);
                LOGV(2, "%s encode frame %zu ok -> %u bytes (%s)", s->id, f, ab, r.backend == VOC_BE_CHIP ? "chip" : "sw");
                size_t m = rtp_build(out, sizeof out, rv.marker && f == 0, s->output_pt, s->out_seq++, s->out_ts, s->out_ssrc, ambe, ab);
                s->out_ts += 160;
                rtp_send(s, &st, out, m);
            }
        }
    }
    fprintf(stderr, "[RTP] %s stream ended: ", s->id);
    rtp_report(s, &st);
    return NULL;
}

/* ---- stream lifecycle ---------------------------------------------------- */
static void stream_init(struct slc_stream *s)
{
    memset(s, 0, sizeof *s);
    s->in_fd = s->out_fd = -1;
    pthread_mutex_init(&s->lock, NULL);
}

static void stream_stop(struct slc_stream *s, bool notify)
{
    pthread_mutex_lock(&s->lock);
    bool was = s->active;
    s->active = false;
    pthread_mutex_unlock(&s->lock);
    if (was) {
        shutdown(s->in_fd, SHUT_RDWR);       /* wakes the media thread's recv() */
        pthread_join(s->thread, NULL);
        voc_stream_release(s->voc);          /* free the chip for others at once */
        if (notify && s->control_fd >= 0) send_state(s->control_fd, s->id, false);
    }
}

static void stream_close(struct slc_stream *s, bool notify)
{
    stream_stop(s, notify);
    if (s->in_fd >= 0) close(s->in_fd);
    if (s->out_fd >= 0) close(s->out_fd);
    voc_stream_destroy(s->voc);
    s->voc = NULL;
    s->in_fd = s->out_fd = -1;
    s->open = false;
}

/* The control connection's peer address, as a string (any family). */
static void control_peer(int fd, char *buf, size_t len)
{
    struct sockaddr_storage ss;
    socklen_t sl = sizeof ss;
    snprintf(buf, len, "?");
    if (getpeername(fd, (struct sockaddr *)&ss, &sl) != 0) return;
    int port;
    addr_str(&ss, buf, len, &port);
    if (!strncmp(buf, "::ffff:", 7)) memmove(buf, buf + 7, strlen(buf + 7) + 1);  /* v4-mapped */
}

/* Called only from the command handler (serialised by handle_lock). */
static struct slc_stream *stream_alloc(struct slc_server *srv)
{
    for (int i = 0; i < SLC_MAX_STREAMS; i++) if (!srv->streams[i].open) return &srv->streams[i];
    return NULL;
}

static struct slc_stream *stream_find(struct slc_server *srv, const char *sid)
{
    for (int i = 0; i < SLC_MAX_STREAMS; i++)
        if (srv->streams[i].open && !strcmp(srv->streams[i].id, sid)) return &srv->streams[i];
    return NULL;
}

/* The "busy" error says who holds the slots, so a leak is diagnosable from
 * the client's log alone: which peer, which conversions, whether they carry
 * any traffic. Also logged on the daemon side. */
static json_object *busy_error(struct slc_server *srv, const char *id)
{
    char msg[2048];
    size_t used = (size_t)snprintf(msg, sizeof msg, "all %d codec streams are in use:", SLC_MAX_STREAMS);
    int64_t now = now_ms();
    for (int i = 0; i < SLC_MAX_STREAMS && used < sizeof msg; i++) {
        struct slc_stream *s = &srv->streams[i];
        int64_t last = s->last_frame_ms;
        char age[48];
        if (!s->active)  snprintf(age, sizeof age, "not started");
        else if (!last)  snprintf(age, sizeof age, "no traffic yet");
        else             snprintf(age, sizeof age, "last frame %llds ago", (long long)((now - last) / 1000));
        int w = snprintf(msg + used, sizeof msg - used, " %s %s->%s from %s (%s, open %llds);",
                         s->id, s->from, s->to, s->peer, age, (long long)((now - s->opened_ms) / 1000));
        if (w < 0) break;
        used += (size_t)w;
    }
    fprintf(stderr, "[busy] %s\n", msg);
    return resp_err(id, "busy", msg);
}

/* ---- command handling ---------------------------------------------------- */
static const char *jget_str(json_object *o, const char *k)
{
    json_object *v = NULL;
    return (o && json_object_object_get_ex(o, k, &v)) ? json_object_get_string(v) : NULL;
}

static json_object *cmd_hello(struct slc_server *srv, const char *id, json_object *jp, bool *hello)
{
    json_object *vers = NULL;
    if (!jp || !json_object_object_get_ex(jp, "supported_protocol_versions", &vers) || json_object_get_type(vers) != json_type_array)
        return resp_err(id, "missing_parameter", "supported_protocol_versions required");
    bool ok = false;
    for (size_t i = 0; i < json_object_array_length(vers); i++)
        if (!strcmp(json_object_get_string(json_object_array_get_idx(vers, i)), "1.0")) ok = true;
    if (!ok) return resp_err(id, "unsupported_protocol_version", "server supports protocol 1.0");
    LOGV(1, "hello ok — client speaks protocol 1.0");
    *hello = true;
    json_object *r = json_object_new_object();
    json_object_object_add(r, "protocol_name", jstr("SDR-SLC Control Plane"));
    json_object_object_add(r, "selected_protocol_version", jstr("1.0"));
    json_object_object_add(r, "device_name", jstr(srv->name));
    json_object_object_add(r, "manufacturer", jstr("IXChange BV"));
    json_object_object_add(r, "firmware_version", jstr("sdr-slc-vocoderd " SLC_VERSION));
    return resp_ok(id, r);
}

static json_object *cmd_open_stream(struct slc_server *srv, int fd, const char *id, json_object *jp)
{
    struct slc_stream *s = stream_alloc(srv);
    if (!s) return busy_error(srv, id);

    const char *function = jget_str(jp, "function");
    if (!function || strcmp(function, "codec")) return resp_err(id, "not_supported", "function must be codec");
    const char *protocol = jget_str(jp, "protocol");
    if (!protocol || strcmp(protocol, "rtp")) return resp_err(id, "unsupported_stream_protocol", "only RTP is supported");
    const char *from = jget_str(jp, "from"), *to = jget_str(jp, "to");
    if (!from || !to) return resp_err(id, "missing_parameter", "from and to are required");

    static const struct { const char *name; voc_fmt_t fmt; } FMTS[] = {
        { "ambe-dmr", VOC_FMT_DMR }, { "ambe-ysf", VOC_FMT_YSF }, { "ambe-dstar", VOC_FMT_DSTAR } };
    int dec = -1;
    voc_fmt_t fmt = VOC_FMT_DMR;
    for (size_t i = 0; i < sizeof FMTS / sizeof FMTS[0]; i++) {
        if (!strcmp(from, FMTS[i].name) && !strcmp(to, "pcm8k")) { dec = 1; fmt = FMTS[i].fmt; }
        if (!strcmp(from, "pcm8k") && !strcmp(to, FMTS[i].name)) { dec = 0; fmt = FMTS[i].fmt; }
    }
    if (dec < 0) return resp_err(id, "unsupported_conversion", "conversion not advertised");

    json_object *jrtp = NULL, *jout = NULL, *jdest = NULL, *jip = NULL, *jport = NULL, *jin = NULL, *jfpp = NULL, *jofpp = NULL;
    if (!json_object_object_get_ex(jp, "rtp", &jrtp) || !json_object_object_get_ex(jrtp, "output", &jout) ||
        !json_object_object_get_ex(jout, "destination", &jdest) || !json_object_object_get_ex(jdest, "ip", &jip) ||
        !json_object_object_get_ex(jdest, "port", &jport))
        return resp_err(id, "missing_parameter", "rtp.output.destination required");
    unsigned infpp = 1, outfpp = 1;
    if (json_object_object_get_ex(jrtp, "input", &jin) && json_object_object_get_ex(jin, "frames_per_packet", &jfpp))
        infpp = (unsigned)json_object_get_int(jfpp);
    if (json_object_object_get_ex(jout, "frames_per_packet", &jofpp)) outfpp = (unsigned)json_object_get_int(jofpp);
    if (!infpp || !outfpp) return resp_err(id, "invalid_parameter", "frames_per_packet must be >=1");

    uint16_t inport = 0;
    s->in_fd = bind_udp(&inport);
    s->out_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s->in_fd < 0 || s->out_fd < 0) { stream_close(s, false); return resp_err(id, "internal_error", "could not create RTP sockets"); }

    const char *decl = json_object_get_string(jip);
    int dport = json_object_get_int(jport);
    control_peer(fd, s->peer, sizeof s->peer);
    /* Diagnostic only: the daemon cannot know the client's netmask, so a
     * destination that differs from the control peer is not "fixed" here. */
    if (strcmp(decl, s->peer) != 0)
        LOGV(1, "open_stream: RTP destination %s differs from control peer %s — fine if they share a subnet", decl, s->peer);
    if (resolve_dest(decl, dport, &s->output_addr, &s->output_addr_len) < 0) {
        stream_close(s, false);
        return resp_err(id, "invalid_parameter", "invalid output destination");
    }

    s->decode = dec;
    s->fmt = fmt;
    s->input_pt = 97;
    s->output_pt = 96;
    s->input_fpp = infpp;
    s->output_fpp = outfpp;
    s->out_seq = (uint16_t)rnd32();
    s->out_ts = rnd32();
    s->out_ssrc = rnd32();
    s->control_fd = fd;
    s->opened_ms = now_ms();
    s->last_frame_ms = 0;
    snprintf(s->id, sizeof s->id, "cv-%08x", rnd32());
    snprintf(s->from, sizeof s->from, "%s", from);
    snprintf(s->to, sizeof s->to, "%s", to);
    s->voc = voc_stream_create(fmt, dec, s->id);
    if (!s->voc) { stream_close(s, false); return resp_err(id, "internal_error", "vocoder allocation failed"); }
    s->open = true;

    char lip[INET6_ADDRSTRLEN], aip[INET6_ADDRSTRLEN];
    int aport;
    local_ip_for(&s->output_addr, lip);
    addr_str(&s->output_addr, aip, sizeof aip, &aport);
    LOGV(1, "open_stream %s: %s → %s (%s) | client sends to our %s:%u PT %u | we send to %s:%d PT %u | in_fpp=%u out_fpp=%u",
         s->id, from, to, dec ? "DECODE ambe→pcm" : "ENCODE pcm→ambe", lip, (unsigned)inport, (unsigned)s->input_pt,
         aip, aport, (unsigned)s->output_pt, infpp, outfpp);

    json_object *r = json_object_new_object(), *rr = json_object_new_object(), *ri = json_object_new_object(),
                *ro = json_object_new_object(), *recvj = json_object_new_object(), *destj = json_object_new_object();
    json_object_object_add(r, "stream_id", jstr(s->id));
    json_object_object_add(r, "function", jstr("codec"));
    json_object_object_add(r, "from", jstr(from));
    json_object_object_add(r, "to", jstr(to));
    json_object_object_add(recvj, "ip", jstr(lip));
    json_object_object_add(recvj, "port", json_object_new_int(inport));
    json_object_object_add(ri, "receive", recvj);
    json_object_object_add(ri, "payload_type", json_object_new_int(s->input_pt));
    json_object_object_add(ri, "clock_rate", json_object_new_int(8000));
    json_object_object_add(ri, "frames_per_packet", json_object_new_int(infpp));
    json_object_object_add(destj, "ip", jstr(decl));
    json_object_object_add(destj, "port", json_object_new_int(dport));
    json_object_object_add(ro, "destination", destj);
    json_object_object_add(ro, "payload_type", json_object_new_int(s->output_pt));
    json_object_object_add(ro, "clock_rate", json_object_new_int(8000));
    json_object_object_add(ro, "frames_per_packet", json_object_new_int(outfpp));
    json_object_object_add(rr, "input", ri);
    json_object_object_add(rr, "output", ro);
    json_object_object_add(r, "rtp", rr);
    return resp_ok(id, r);
}

static json_object *handle(struct slc_server *srv, int fd, json_object *req, bool *hello)
{
    json_object *jp = NULL;
    json_object_object_get_ex(req, "params", &jp);
    const char *id = jget_str(req, "id");
    const char *cmd = jget_str(req, "command");
    if (!id) id = "";
    if (!cmd) return resp_err(id, "invalid_request", "missing command");
    if (strcmp(cmd, "ping") != 0) LOGV(1, "cmd '%s' id='%s'", cmd, id);   /* pings every 10 s: -vv only */
    else LOGV(2, "cmd 'ping' id='%s'", id);
    if (strcmp(cmd, "hello") != 0 && !*hello) return resp_err(id, "invalid_request", "hello must be first");

    if (!strcmp(cmd, "hello")) return cmd_hello(srv, id, jp, hello);
    if (!strcmp(cmd, "get_capabilities")) return resp_ok(id, capabilities());
    /* ping (SDR-SLC-CP envelope command): liveness only, no state change. */
    if (!strcmp(cmd, "ping")) return resp_ok(id, NULL);
    if (!strcmp(cmd, "open_stream")) return cmd_open_stream(srv, fd, id, jp);

    if (!strcmp(cmd, "start_codec") || !strcmp(cmd, "stop_codec") || !strcmp(cmd, "close_stream")) {
        const char *sid = jget_str(jp, "stream_id");
        if (!sid) return resp_err(id, "missing_parameter", "stream_id required");
        struct slc_stream *s = stream_find(srv, sid);
        if (!s) return resp_err(id, "stream_not_found", "unknown stream_id");

        if (!strcmp(cmd, "start_codec")) {
            if (!s->active) {
                voc_stream_reset(s->voc);
                s->last_frame_ms = 0;
                LOGV(1, "start_codec %s: vocoder reset, media thread starting", s->id);
                s->active = true;
                if (pthread_create(&s->thread, NULL, media_thread, s) != 0) {
                    s->active = false;
                    return resp_err(id, "internal_error", "could not start media thread");
                }
            }
            json_object *r = json_object_new_object();
            json_object_object_add(r, "function", jstr("codec"));
            json_object_object_add(r, "streaming", json_object_new_boolean(1));
            json_object *o = resp_ok(id, r);
            send_json(fd, o);
            json_object_put(o);
            send_state(fd, s->id, true);
            return NULL;
        }
        if (!strcmp(cmd, "stop_codec")) {
            LOGV(1, "stop_codec %s", s->id);
            stream_stop(s, true);
            json_object *r = json_object_new_object();
            json_object_object_add(r, "function", jstr("codec"));
            json_object_object_add(r, "streaming", json_object_new_boolean(0));
            return resp_ok(id, r);
        }
        LOGV(1, "close_stream %s", s->id);
        stream_close(s, true);
        json_object *r = json_object_new_object();
        json_object_object_add(r, "closed", json_object_new_boolean(1));
        return resp_ok(id, r);
    }
    return resp_err(id, "unknown_command", "command not implemented by codec Tier 1");
}

/* Commands are serialised so concurrent control connections cannot race on
 * the stream pool; the connections themselves are concurrent. */
static pthread_mutex_t handle_lock = PTHREAD_MUTEX_INITIALIZER;

static void client_loop(struct slc_server *srv, int fd)
{
    bool hello = false;
    char peer[64];
    control_peer(fd, peer, sizeof peer);
    LOGV(1, "control connection from %s", peer);
    for (;;) {
        uint32_t be;
        if (recv_all(fd, &be, 4) <= 0) break;
        uint32_t n = ntohl(be);
        if (n > SLC_MAX_JSON) break;
        char *buf = malloc((size_t)n + 1);
        if (!buf) break;
        if (recv_all(fd, buf, n) <= 0) { free(buf); break; }
        buf[n] = 0;
        json_object *req = json_tokener_parse(buf);
        free(buf);
        if (!req) {
            json_object *e = resp_err("", "invalid_request", "invalid JSON");
            send_json(fd, e);
            json_object_put(e);
            continue;
        }
        pthread_mutex_lock(&handle_lock);
        json_object *resp = handle(srv, fd, req, &hello);
        pthread_mutex_unlock(&handle_lock);
        json_object_put(req);
        if (resp) {
            int rc = send_json(fd, resp);
            json_object_put(resp);
            if (rc < 0) break;
        }
    }
    /* Tear down only the streams THIS connection opened: a capability reader
     * that connects, asks and leaves must not stop someone else's stream. */
    int closed = 0;
    pthread_mutex_lock(&handle_lock);
    for (int i = 0; i < SLC_MAX_STREAMS; i++)
        if (srv->streams[i].open && srv->streams[i].control_fd == fd) { stream_close(&srv->streams[i], false); closed++; }
    pthread_mutex_unlock(&handle_lock);
    if (closed) fprintf(stderr, "[ctl] connection from %s closed; released %d stream%s\n", peer, closed, closed == 1 ? "" : "s");
    close(fd);
}

struct client_arg { struct slc_server *srv; int fd; };
static void *client_thread(void *p)
{
    struct client_arg *a = p;
    client_loop(a->srv, a->fd);
    free(a);
    return NULL;
}

/* ---- main ---------------------------------------------------------------- */
static void usage(const char *argv0)
{
    printf("usage: %s [options]\n"
           "  --port N               SLC-CP TCP port (default 4630)\n"
           "  --name NAME            device name in mDNS and hello (default SLC-VOCODER-<host>)\n"
           "\n"
           "AMBE-3000 (DVstick30 / ThumbDV):\n"
           "  --no-hw                never use an AMBE-3000; mbelib-neo for everything\n"
           "  --hw-dev PATH          use PATH (e.g. /dev/ttyUSB0) instead of scanning USB\n"
           "  --hw-canonical         the chip uses DVSI canonical DMR bit order instead of\n"
           "                         on-air order (not needed for a DVstick30 at the standard\n"
           "                         DMR rate; alias: --hw-deinterleave)\n"
           "  --hw-decode-gain-db N  chip decode output gain (default 0)\n"
           "  --hw-encode-gain-db N  chip encode input gain (default 0)\n"
           "\n"
           "Software vocoder (mbelib-neo), level-matched to the chip by design:\n"
           "  --decode-gain-db DB    gain on software-decoded audio (default 0)\n"
           "  --encode-gain-db DB    gain on audio into the software encoder (default 0)\n"
           "\n"
           "Chip sharing (who gets the AMBE-3000):\n"
           "  --tx-vad-dbfs DB       transmit claims the chip above this frame level (default -50)\n"
           "  --tx-hang-ms MS        transmit keeps it this long after the last voice (default 2000)\n"
           "  --rx-hang-ms MS        receive keeps it this long after the last good frame (default 300)\n"
           "  --warmup-frames N      silent chip frames before a receive crossfade (default 3)\n"
           "\n"
           "  -v                     control plane and chip ownership changes\n"
           "  -vv                    also every frame (high volume)\n", argv0);
}

int main(int argc, char **argv)
{
    struct slc_server srv = {0};
    srv.port = SLC_DEFAULT_PORT;
    char host[64] = "linux";
    gethostname(host, sizeof host - 1);
    snprintf(srv.name, sizeof srv.name, "SLC-VOCODER-%s", host);

    struct voc_config cfg;
    voc_config_defaults(&cfg);
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *v = (i + 1 < argc) ? argv[i + 1] : NULL;
#define ARG(name) (!strcmp(a, name) && v && (i++, 1))
        if      (ARG("--port"))               srv.port = (uint16_t)atoi(v);
        else if (ARG("--name"))               snprintf(srv.name, sizeof srv.name, "%s", v);
        else if (!strcmp(a, "--no-hw"))       cfg.disable_hw = 1;
        else if (ARG("--hw-dev"))             cfg.hw_dev = v;
        else if (!strcmp(a, "--hw-canonical") || !strcmp(a, "--hw-deinterleave")) cfg.hw_canonical = 1;
        else if (ARG("--hw-decode-gain-db"))  cfg.hw_out_gain_db = atoi(v);
        else if (ARG("--hw-encode-gain-db"))  cfg.hw_in_gain_db = atoi(v);
        else if (ARG("--decode-gain-db"))     cfg.sw_dec_gain = powf(10.f, (float)atof(v) / 20.f);
        else if (ARG("--encode-gain-db"))     cfg.sw_enc_gain = powf(10.f, (float)atof(v) / 20.f);
        else if (ARG("--tx-vad-dbfs"))        cfg.tx_vad_dbfs = (float)atof(v);
        else if (ARG("--tx-hang-ms"))         cfg.tx_hang_ms = atoi(v);
        else if (ARG("--rx-hang-ms"))         cfg.rx_hang_ms = atoi(v);
        else if (ARG("--warmup-frames"))      cfg.warmup_frames = atoi(v);
        else if (!strcmp(a, "-v") || !strcmp(a, "--verbose")) slc_verbose++;
        else if (!strcmp(a, "-vv"))           slc_verbose += 2;
        else if (!strcmp(a, "--help") || !strcmp(a, "-h")) { usage(argv[0]); return 0; }
        else { fprintf(stderr, "unknown or incomplete option: %s (see --help)\n", a); return 2; }
#undef ARG
    }
    cfg.verbose = slc_verbose;

    for (int i = 0; i < SLC_MAX_STREAMS; i++) stream_init(&srv.streams[i]);
    /* Shutdown signals. In a threaded process the kernel may deliver SIGTERM
     * to any thread that does not block it, e.g. a media thread sitting in
     * recv(), and the main thread would then never leave accept(). So the
     * signals are blocked here, before any thread exists (threads inherit the
     * mask), and the main loop alone receives them, atomically, in ppoll(). */
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
    sigset_t shutdown_sigs, wait_mask;
    sigemptyset(&shutdown_sigs);
    sigaddset(&shutdown_sigs, SIGINT);
    sigaddset(&shutdown_sigs, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &shutdown_sigs, &wait_mask);
    sigdelset(&wait_mask, SIGINT);
    sigdelset(&wait_mask, SIGTERM);

    /* Prefer a dual-stack IPv6 listener; fall back to IPv4 where IPv6 is
     * unavailable (containers, minimal kernels). */
    int one = 1;
    srv.listen_fd = socket(AF_INET6, SOCK_STREAM, 0);
    if (srv.listen_fd >= 0) {
        int zero = 0;
        setsockopt(srv.listen_fd, IPPROTO_IPV6, IPV6_V6ONLY, &zero, sizeof zero);
        setsockopt(srv.listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        struct sockaddr_in6 a = {0};
        a.sin6_family = AF_INET6;
        a.sin6_addr = in6addr_any;
        a.sin6_port = htons(srv.port);
        if (bind(srv.listen_fd, (struct sockaddr *)&a, sizeof a) < 0 || listen(srv.listen_fd, 8) < 0) {
            close(srv.listen_fd);
            srv.listen_fd = -1;
        }
    }
    if (srv.listen_fd < 0) {
        srv.listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (srv.listen_fd < 0) { perror("socket"); return 1; }
        setsockopt(srv.listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        struct sockaddr_in a4 = {0};
        a4.sin_family = AF_INET;
        a4.sin_addr.s_addr = htonl(INADDR_ANY);
        a4.sin_port = htons(srv.port);
        if (bind(srv.listen_fd, (struct sockaddr *)&a4, sizeof a4) < 0 || listen(srv.listen_fd, 8) < 0) { perror("bind/listen"); return 1; }
        LOGV(1, "IPv6 unavailable — listening on IPv4 only");
    }

    if (voc_init(&cfg) < 0) { fprintf(stderr, "vocoder: initialisation failed\n"); return 1; }
    mdns_start(srv.name, srv.port);
    fprintf(stderr, "sdr-slc-vocoderd %s: SLC-CP on TCP %u as \"%s\", %d streams, software vocoder %s (verbose=%d)\n",
            SLC_VERSION, srv.port, srv.name, SLC_MAX_STREAMS, voc_sw_version(), slc_verbose);

    while (!quitting) {
        struct pollfd pfd = { srv.listen_fd, POLLIN, 0 };
        int pr = ppoll(&pfd, 1, NULL, &wait_mask);   /* the only place signals are taken */
        if (pr < 0) { if (errno == EINTR) continue; perror("ppoll"); break; }
        struct sockaddr_storage peer;
        socklen_t pn = sizeof peer;
        int fd = accept(srv.listen_fd, (struct sockaddr *)&peer, &pn);
        if (fd < 0) { if (errno == EINTR || errno == EAGAIN || errno == ECONNABORTED) continue; perror("accept"); break; }
        set_keepalive(fd);
        struct client_arg *ca = malloc(sizeof *ca);
        if (!ca) { close(fd); continue; }
        ca->srv = &srv;
        ca->fd = fd;
        pthread_t th;
        if (pthread_create(&th, NULL, client_thread, ca) != 0) { free(ca); close(fd); continue; }
        pthread_detach(th);
    }
    fprintf(stderr, "sdr-slc-vocoderd: shutting down\n");
    mdns_stop();
    pthread_mutex_lock(&handle_lock);
    for (int i = 0; i < SLC_MAX_STREAMS; i++) stream_close(&srv.streams[i], false);
    pthread_mutex_unlock(&handle_lock);
    voc_shutdown();
    close(srv.listen_fd);
    return 0;
}
