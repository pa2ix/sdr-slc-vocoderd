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

/* ambe3k.c - see ambe3k.h.
 *
 * DVSI AMBE-3000 packet protocol (all multi-byte fields big-endian):
 *
 *   [0x61][len_hi][len_lo][type]  <payload: len bytes>
 *
 *   type 0x00 CONTROL, 0x01 CHANNEL (AMBE data), 0x02 SPEECH (PCM).
 *   len counts everything AFTER the 4-byte header.
 *
 * DMR (3600 bps, 2450x1150 FEC, 72 bits / 9 bytes):
 *   RATEP  61 00 0D 00 0A 04 31 07 54 24 00 00 00 00 00 6F 48
 * D-STAR (3600 bps, 2400x1200 FEC, 72 bits / 9 bytes):
 *   RATEP  61 00 0D 00 0A 01 30 07 63 40 00 00 00 00 00 00 48
 * Both rates:
 *   encode 61 01 42 02 00 A0 <320 PCM bytes, samples big-endian>  -> CHANNEL
 *   decode 61 00 0B 01 01 48 <9 AMBE bytes>                       -> SPEECH
 *   prodid 61 00 01 00 30                                         -> CONTROL/0x30
 *
 * The two RATEP parameter sets are the ones every open D-STAR/DMR gateway
 * that drives a DV3000/AMBE-3000 uses (MMDVMHost, ircDDBGateway, AMBEServer,
 * serialDV), i.e. the DVSI "rate 33" (D-STAR) and the DMR 2450x1150 set.
 */
#define _POSIX_C_SOURCE 200809L
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE            /* cfmakeraw, CRTSCTS, Linux serial ioctls */
#endif
#include "ambe3k.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifdef __linux__
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif

/* ---- protocol constants -------------------------------------------------- */
#define DV_START        0x61u
#define DV_TYPE_CONTROL 0x00u
#define DV_TYPE_CHANNEL 0x01u   /* AMBE data */
#define DV_TYPE_SPEECH  0x02u   /* PCM       */
#define DV_CTL_RATEP    0x0Au
#define DV_CTL_GAIN     0x4Bu
#define DV_CTL_PRODID   0x30u

static const uint8_t REQ_PRODID[]  = { DV_START, 0x00, 0x01, DV_TYPE_CONTROL, DV_CTL_PRODID };
static const uint8_t REQ_DMR_RATEP[] = { DV_START, 0x00, 0x0D, DV_TYPE_CONTROL, DV_CTL_RATEP,
                                         0x04, 0x31, 0x07, 0x54, 0x24, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x6F, 0x48 };
static const uint8_t REQ_DSTAR_RATEP[] = { DV_START, 0x00, 0x0D, DV_TYPE_CONTROL, DV_CTL_RATEP,
                                         0x01, 0x30, 0x07, 0x63, 0x40, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x48 };
static const uint8_t HDR_SPEECH[]  = { DV_START, 0x01, 0x42, DV_TYPE_SPEECH,  0x00, 0xA0 };
static const uint8_t HDR_CHANNEL[] = { DV_START, 0x00, 0x0B, DV_TYPE_CHANNEL, 0x01, 0x48 };

/* Timeouts. A codec transaction moves ~340 bytes at 460800 baud (~7.4 ms)
 * plus USB latency (~1-2 ms with the FTDI latency timer at 1 ms), so 80 ms
 * is ample margin while still bounding how long one sick exchange can delay
 * the owning stream. Control packets (identify, rate, gain) are rare and get
 * the old generous timeout. */
#define CTL_TIMEOUT_MS   250
#define CODEC_TIMEOUT_MS  80
#define RX_BUF_MAX       512   /* biggest payload is 322 (speech) + slack  */

struct ambe3k {
    int  fd;
    char path[PATH_MAX];
    char prodid[AMBE3K_PRODID_MAX];
    ambe3k_rate_t rate;                /* rate last accepted by the chip */
};

/* ---- low-level serial ---------------------------------------------------- */

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static int write_all(int fd, const void *buf, size_t n)
{
    const uint8_t *p = buf;
    while (n) {
        ssize_t w = write(fd, p, n);
        if (w < 0) { if (errno == EINTR) continue; return -1; }
        p += w; n -= (size_t)w;
    }
    return 0;
}

/* An errno that means the device itself is gone (unplugged, USB reset). */
static int errno_is_gone(int e)
{
    return e == EIO || e == ENODEV || e == ENXIO || e == EBADF || e == ENOENT;
}

/* Read exactly n bytes. AMBE3K_ETIMEOUT on timeout, AMBE3K_EGONE if the
 * device disappeared (EOF on a tty, or a device-gone errno). */
static int read_exact(int fd, void *buf, size_t n, long deadline)
{
    uint8_t *p = buf;
    while (n) {
        long remain = deadline - now_ms();
        if (remain <= 0) return AMBE3K_ETIMEOUT;
        fd_set rs; FD_ZERO(&rs); FD_SET(fd, &rs);
        struct timeval tv = { remain / 1000, (remain % 1000) * 1000 };
        int r = select(fd + 1, &rs, NULL, NULL, &tv);
        if (r < 0) { if (errno == EINTR) continue; return errno_is_gone(errno) ? AMBE3K_EGONE : AMBE3K_ETIMEOUT; }
        if (r == 0) return AMBE3K_ETIMEOUT;
        ssize_t got = read(fd, p, n);
        if (got < 0) { if (errno == EINTR || errno == EAGAIN) continue; return errno_is_gone(errno) ? AMBE3K_EGONE : AMBE3K_ETIMEOUT; }
        if (got == 0) return AMBE3K_EGONE;     /* hangup: the tty went away */
        p += got; n -= (size_t)got;
    }
    return 0;
}

/* Receive one framed packet. Hunts for the start byte to resync past noise,
 * then reads header + payload. On success *type is the packet type and the
 * payload (the bytes after the 4-byte header) is in payload[0..*plen). */
static int recv_frame_to(int fd, uint8_t *type, uint8_t *payload,
                         size_t cap, size_t *plen, long timeout_ms)
{
    long deadline = now_ms() + timeout_ms;
    int rc;

    /* resync on start byte */
    for (;;) {
        uint8_t b;
        if ((rc = read_exact(fd, &b, 1, deadline)) < 0) return rc;
        if (b == DV_START) break;
    }
    uint8_t h[3];                              /* len_hi, len_lo, type */
    if ((rc = read_exact(fd, h, 3, deadline)) < 0) return rc;
    size_t len = (size_t)h[0] << 8 | h[1];
    *type = h[2];
    if (len > cap) return AMBE3K_EPROTO;
    if (len && (rc = read_exact(fd, payload, len, deadline)) < 0) return rc;
    *plen = len;
    return 0;
}

static int recv_frame(int fd, uint8_t *type, uint8_t *payload, size_t cap, size_t *plen)
{
    return recv_frame_to(fd, type, payload, cap, plen, CTL_TIMEOUT_MS);
}

/* Send one request and receive its reply, robustly.
 *
 * The input queue is flushed FIRST. The chip answers every request with
 * exactly one packet, so anything already waiting is a reply that arrived
 * after its own transaction gave up (or power-on chatter). sdr-slc-codecd
 * did not flush, and one late reply then shifted every later exchange by one
 * packet: a RATEP read a SPEECH packet, a decode read the RATEP ack, and the
 * chip looked broken (the stream fell back to software) until restart.
 * Flushing costs one ioctl and makes each transaction self-contained.
 *
 * On any failure the input is flushed again, so a reply that is still on its
 * way cannot be mistaken for the next transaction's.
 * Returns 0, or AMBE3K_ETIMEOUT / AMBE3K_EGONE / AMBE3K_EPROTO. */
static int transact(int fd, const uint8_t *req, size_t reqlen, uint8_t want_type,
                    uint8_t *payload, size_t cap, size_t *plen, long timeout_ms)
{
    if (tcflush(fd, TCIFLUSH) != 0 && errno_is_gone(errno)) return AMBE3K_EGONE;
    if (write_all(fd, req, reqlen) < 0) return errno_is_gone(errno) ? AMBE3K_EGONE : AMBE3K_ETIMEOUT;
    uint8_t type;
    int rc = recv_frame_to(fd, &type, payload, cap, plen, timeout_ms);
    if (rc == 0 && type != want_type) rc = AMBE3K_EPROTO;
    if (rc < 0 && rc != AMBE3K_EGONE) (void)tcflush(fd, TCIFLUSH);
    return rc;
}

/* Open the port raw at 460800 8N1, no flow control, and drop the FTDI
 * latency timer so small reads don't stall ~16 ms. Returns fd or -1. */
static int serial_open(const char *path)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) return -1;

    struct termios t;
    if (tcgetattr(fd, &t) != 0) { close(fd); return -1; }
    cfmakeraw(&t);
    cfsetispeed(&t, B460800);
    cfsetospeed(&t, B460800);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~CRTSCTS;                     /* no hardware flow control */
    t.c_cflag &= ~CSTOPB;                      /* 1 stop bit */
    t.c_cflag &= ~PARENB;                      /* no parity  */
    t.c_iflag &= ~(IXON | IXOFF | IXANY);      /* no software flow control */
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;                         /* we time out via select() */
    if (tcsetattr(fd, TCSANOW, &t) != 0) { close(fd); return -1; }

#ifdef __linux__
    struct serial_struct ss;
    if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
        ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_LOW_LATENCY;
        (void)ioctl(fd, TIOCSSERIAL, &ss);     /* best effort; ignore failure */
    }
#endif
    tcflush(fd, TCIOFLUSH);                     /* clear any power-on banter  */
    return fd;
}

/* ---- product-id probe ---------------------------------------------------- */

int ambe3k_probe(const char *devpath, char *prodid, size_t n)
{
    int fd = serial_open(devpath);
    if (fd < 0) return -1;

    if (write_all(fd, REQ_PRODID, sizeof REQ_PRODID) < 0) { close(fd); return -1; }

    uint8_t type, pl[RX_BUF_MAX]; size_t pn;
    int rc = recv_frame(fd, &type, pl, sizeof pl, &pn);
    close(fd);
    if (rc < 0) return -1;

    /* CONTROL response, field 0x30, then ASCII id (NUL-terminated). */
    if (type != DV_TYPE_CONTROL || pn < 2 || pl[0] != DV_CTL_PRODID) return 0;

    char id[AMBE3K_PRODID_MAX] = {0};
    size_t k = 0;
    for (size_t i = 1; i < pn && k + 1 < sizeof id; i++) {
        if (pl[i] == 0) break;
        if (pl[i] >= 32 && pl[i] < 127) id[k++] = (char)pl[i];
    }
    id[k] = 0;
    if (prodid && n) { snprintf(prodid, n, "%s", id); }
    return strstr(id, "AMBE3000") ? 1 : (strstr(id, "AMBE") ? 1 : 0);
}

/* ---- sysfs discovery ----------------------------------------------------- */

static const struct { unsigned vid, pid; } BRIDGES[] = {
    { 0x0403, 0x6015 },   /* FTDI FT-X / FT230X  (current DVstick30)     */
    { 0x0403, 0x6001 },   /* FTDI FT232R  (older AMBE sticks)            */
    { 0x0403, 0x6014 },   /* FTDI FT232H                                 */
    { 0x0403, 0x6010 },   /* FTDI FT2232                                 */
    { 0x10c4, 0xea60 },   /* Silicon Labs CP2102 (alternate build)       */
};

static int read_hex_line(const char *path, unsigned *out)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char buf[64];
    char *p = fgets(buf, sizeof buf, f);
    fclose(f);
    if (!p) return -1;
    *out = (unsigned)strtoul(buf, NULL, 16);
    return 0;
}

/* Walk up from /sys/class/tty/<name>/device until idVendor appears. */
static int tty_usb_ids(const char *name, unsigned *vid, unsigned *pid)
{
    char link[PATH_MAX], real[PATH_MAX], probe[PATH_MAX + 32];
    snprintf(link, sizeof link, "/sys/class/tty/%s/device", name);
    if (!realpath(link, real)) return -1;
    for (;;) {
        snprintf(probe, sizeof probe, "%s/idVendor", real);
        if (access(probe, R_OK) == 0) {
            if (read_hex_line(probe, vid) < 0) return -1;
            snprintf(probe, sizeof probe, "%s/idProduct", real);
            if (read_hex_line(probe, pid) < 0) return -1;
            return 0;
        }
        char *slash = strrchr(real, '/');
        if (!slash || slash == real) return -1;   /* reached root */
        *slash = 0;
    }
}

static int is_bridge(unsigned vid, unsigned pid)
{
    for (size_t i = 0; i < sizeof BRIDGES / sizeof BRIDGES[0]; i++)
        if (BRIDGES[i].vid == vid && BRIDGES[i].pid == pid) return 1;
    return 0;
}

int ambe3k_candidates(char *buf, size_t len)
{
    if (buf && len) buf[0] = 0;
    DIR *d = opendir("/sys/class/tty");
    if (!d) return 0;
    /* Collect, then sort, so the signature does not depend on readdir order. */
    char names[64][32]; int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL && n < 64) {
        if (strncmp(e->d_name, "ttyUSB", 6) != 0) continue;
        unsigned vid = 0, pid = 0;
        if (tty_usb_ids(e->d_name, &vid, &pid) != 0 || !is_bridge(vid, pid)) continue;
        size_t nl = strnlen(e->d_name, sizeof names[0] - 1);
        memcpy(names[n], e->d_name, nl); names[n][nl] = 0; n++;
    }
    closedir(d);
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && strcmp(names[j - 1], names[j]) > 0; j--) {
            char t[32]; memcpy(t, names[j], 32); memcpy(names[j], names[j - 1], 32); memcpy(names[j - 1], t, 32);
        }
    size_t used = 0;
    for (int i = 0; i < n && buf && used + 1 < len; i++) {
        int w = snprintf(buf + used, len - used, "%s%s", i ? " " : "", names[i]);
        if (w < 0 || (size_t)w >= len - used) break;
        used += (size_t)w;
    }
    return n;
}

ambe3k_status_t ambe3k_find(char *out_path, size_t out_len,
                            char *out_prodid, size_t pn,
                            char *out_hint, size_t hint_len)
{
    if (out_hint && hint_len) out_hint[0] = 0;

    DIR *d = opendir("/sys/class/tty");
    if (!d) return AMBE3K_ABSENT;

    /* Track the most actionable non-FOUND outcome we encounter.
     * Reporting priority: FOUND > PERMISSION > NO_RESPONSE > ABSENT. */
    ambe3k_status_t worst = AMBE3K_ABSENT;
    char seen_path[PATH_MAX] = {0};
    char seen_group[64] = {0};

    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, "ttyUSB", 6) != 0) continue;
        unsigned vid = 0, pid = 0;
        if (tty_usb_ids(e->d_name, &vid, &pid) != 0) continue;
        if (!is_bridge(vid, pid)) continue;

        char dev[PATH_MAX];
        snprintf(dev, sizeof dev, "/dev/%s", e->d_name);

        /* Can't even open it? Almost always a group/permission issue. */
        if (access(dev, R_OK | W_OK) != 0 && errno == EACCES) {
            if (worst < AMBE3K_PERMISSION) {   /* enum order: PERMISSION > NO_RESPONSE > ABSENT */
                worst = AMBE3K_PERMISSION;
                snprintf(seen_path, sizeof seen_path, "%s", dev);
                struct stat st;
                struct group *g = (stat(dev, &st) == 0) ? getgrgid(st.st_gid) : NULL;
                snprintf(seen_group, sizeof seen_group, "%s",
                         (g && g->gr_name) ? g->gr_name : "dialout");
            }
            continue;
        }

        char id[AMBE3K_PRODID_MAX] = {0};
        int r = ambe3k_probe(dev, id, sizeof id);
        if (r == 1) {                          /* it's an AMBE-3000 */
            snprintf(out_path, out_len, "%s", dev);
            if (out_prodid && pn) snprintf(out_prodid, pn, "%s", id);
            closedir(d);
            return AMBE3K_FOUND;
        }
        /* Bridge chip, openable, but not an AMBE-3000 (or silent). */
        if (worst < AMBE3K_NO_RESPONSE) {      /* only downgrade from ABSENT */
            worst = AMBE3K_NO_RESPONSE;
            snprintf(seen_path, sizeof seen_path, "%s", dev);
        }
    }
    closedir(d);

    if (seen_path[0]) snprintf(out_path, out_len, "%s", seen_path);

    if (out_hint && hint_len) {
        if (worst == AMBE3K_PERMISSION)
            snprintf(out_hint, hint_len,
                "a USB serial device is present at %s but it can't be opened "
                "(permission denied). Add the user to the '%s' group "
                "(sudo usermod -aG %s $USER) and restart, or run as root.",
                seen_path, seen_group, seen_group);
        else if (worst == AMBE3K_NO_RESPONSE)
            snprintf(out_hint, hint_len,
                "a USB serial device at %s did not identify as an AMBE-3000. "
                "Check cabling and that no other process holds the port.",
                seen_path);
    }
    return worst;
}

ambe3k_status_t ambe3k_autodetect(ambe3k_t **out_handle,
                                  char *out_path, size_t out_len,
                                  char *out_prodid, size_t pn,
                                  char *out_hint, size_t hint_len)
{
    if (out_handle) *out_handle = NULL;

    ambe3k_status_t st = ambe3k_find(out_path, out_len, out_prodid, pn,
                                     out_hint, hint_len);
    if (st != AMBE3K_FOUND) return st;

    ambe3k_t *dev = ambe3k_open(out_path);
    if (!dev) {
        /* Identified a moment ago but open/config failed (raced away, or the
         * port got grabbed). Treat as no-response so the caller falls back. */
        if (out_hint && hint_len)
            snprintf(out_hint, hint_len,
                "AMBE-3000 at %s was detected but failed to open/configure; "
                "check that nothing else holds the port.", out_path);
        return AMBE3K_NO_RESPONSE;
    }
    if (out_handle) *out_handle = dev;
    else ambe3k_close(dev);
    return AMBE3K_FOUND;
}

/* ---- session ------------------------------------------------------------- */

/* Send a control packet and require the chip to acknowledge `field` with a
 * zero status byte. A CONTROL reply is [field][status]; a non-zero status
 * means the chip rejected the setting (codecd ignored this byte). */
static int control(int fd, const uint8_t *req, size_t reqlen, uint8_t field)
{
    uint8_t pl[RX_BUF_MAX]; size_t pn;
    int rc = transact(fd, req, reqlen, DV_TYPE_CONTROL, pl, sizeof pl, &pn, CTL_TIMEOUT_MS);
    if (rc < 0) return rc;
    if (pn < 1 || pl[0] != field) return AMBE3K_EPROTO;
    if (pn >= 2 && pl[1] != 0) return AMBE3K_EPROTO;
    return 0;
}

ambe3k_t *ambe3k_open(const char *devpath)
{
    ambe3k_t *h = calloc(1, sizeof *h);
    if (!h) return NULL;
    h->fd = serial_open(devpath);
    if (h->fd < 0) { fprintf(stderr, "ambe3k: cannot open %s: %s\n", devpath, strerror(errno)); free(h); return NULL; }
    snprintf(h->path, sizeof h->path, "%s", devpath);

    /* identify */
    {
        uint8_t pl[RX_BUF_MAX]; size_t pn;
        int rc = transact(h->fd, REQ_PRODID, sizeof REQ_PRODID, DV_TYPE_CONTROL, pl, sizeof pl, &pn, CTL_TIMEOUT_MS);
        if (rc == AMBE3K_EGONE) goto fail_io;
        if (rc < 0) {
            fprintf(stderr, "ambe3k: no product-id response from %s\n", devpath);
            goto fail;
        }
        if (pn >= 2 && pl[0] == DV_CTL_PRODID) {
            size_t k = 0;
            for (size_t i = 1; i < pn && k + 1 < sizeof h->prodid; i++) {
                if (pl[i] == 0) break;
                if (pl[i] >= 32 && pl[i] < 127) h->prodid[k++] = (char)pl[i];
            }
            h->prodid[k] = 0;
        }
        if (!strstr(h->prodid, "AMBE")) {
            fprintf(stderr, "ambe3k: %s did not identify as an AMBE device (got \"%s\")\n",
                    devpath, h->prodid);
            goto fail;
        }
    }

    /* configure for DMR (3600 / 2450x1150) - the startup default; the codec
     * arbiter switches to D-STAR on demand via ambe3k_set_rate(). */
    if (control(h->fd, REQ_DMR_RATEP, sizeof REQ_DMR_RATEP, DV_CTL_RATEP) < 0) {
        fprintf(stderr, "ambe3k: DMR rate not accepted by %s\n", devpath);
        goto fail;
    }
    h->rate = AMBE3K_RATE_DMR;

    fprintf(stderr, "ambe3k: %s ready on %s (%s)\n", h->prodid, devpath, ambe3k_rate_name(h->rate));
    return h;

fail_io:
    fprintf(stderr, "ambe3k: I/O error talking to %s: %s\n", devpath, strerror(errno));
fail:
    close(h->fd);
    free(h);
    return NULL;
}

void ambe3k_close(ambe3k_t *h)
{
    if (!h) return;
    if (h->fd >= 0) close(h->fd);
    free(h);
}

const char *ambe3k_prodid(const ambe3k_t *h) { return h ? h->prodid : ""; }
const char *ambe3k_path(const ambe3k_t *h) { return h ? h->path : ""; }

int ambe3k_set_gain(ambe3k_t *h, int in_db, int out_db)
{
    if (!h) return -1;
    if (in_db  < -90) in_db  = -90; else if (in_db  > 90) in_db  = 90;
    if (out_db < -90) out_db = -90; else if (out_db > 90) out_db = 90;
    /* 61 00 03 00 4B <in> <out> : CONTROL/GAIN, signed dB, in then out */
    uint8_t pkt[7] = { DV_START, 0x00, 0x03, DV_TYPE_CONTROL, DV_CTL_GAIN,
                       (uint8_t)(int8_t)in_db, (uint8_t)(int8_t)out_db };
    return control(h->fd, pkt, sizeof pkt, DV_CTL_GAIN);
}

const char *ambe3k_rate_name(ambe3k_rate_t rate)
{
    switch (rate) {
    case AMBE3K_RATE_DSTAR: return "D-STAR 3600/2400x1200";
    case AMBE3K_RATE_DMR:
    default:                return "DMR 3600/2450x1150";
    }
}

ambe3k_rate_t ambe3k_rate(const ambe3k_t *h) { return h ? h->rate : AMBE3K_RATE_DMR; }

int ambe3k_set_rate(ambe3k_t *h, ambe3k_rate_t rate)
{
    if (!h) return -1;
    const uint8_t *pkt; size_t n;
    if (rate == AMBE3K_RATE_DSTAR) { pkt = REQ_DSTAR_RATEP; n = sizeof REQ_DSTAR_RATEP; }
    else                           { pkt = REQ_DMR_RATEP;   n = sizeof REQ_DMR_RATEP;   }
    int rc = control(h->fd, pkt, n, DV_CTL_RATEP);
    if (rc < 0) return rc;
    h->rate = rate;
    return 0;
}

int ambe3k_reset(ambe3k_t *h)
{
    if (!h) return -1;
    /* Re-send the CURRENT rate parameters. Setting RATEP re-initialises the
     * vocoder to a defined state; reusing this validated packet keeps the
     * reset path free of any control field not already exercised on open. */
    return ambe3k_set_rate(h, h->rate);
}

/* ---- codec transactions -------------------------------------------------- */

int ambe3k_encode(ambe3k_t *h, const int16_t pcm[AMBE3K_PCM_SAMPLES],
                  uint8_t ambe9[AMBE3K_AMBE_BYTES])
{
    uint8_t pkt[sizeof HDR_SPEECH + AMBE3K_PCM_SAMPLES * 2];
    memcpy(pkt, HDR_SPEECH, sizeof HDR_SPEECH);
    uint8_t *q = pkt + sizeof HDR_SPEECH;
    for (int i = 0; i < AMBE3K_PCM_SAMPLES; i++) {     /* host -> big-endian */
        uint16_t v = (uint16_t)pcm[i];
        *q++ = (uint8_t)(v >> 8);
        *q++ = (uint8_t)(v & 0xFF);
    }
    uint8_t pl[RX_BUF_MAX]; size_t pn;
    int rc = transact(h->fd, pkt, sizeof pkt, DV_TYPE_CHANNEL, pl, sizeof pl, &pn, CODEC_TIMEOUT_MS);
    if (rc < 0) return rc;
    /* CHANNEL payload = [0x01][0x48][9 bytes] */
    if (pn < 2 + AMBE3K_AMBE_BYTES) { (void)tcflush(h->fd, TCIFLUSH); return AMBE3K_EPROTO; }
    memcpy(ambe9, pl + 2, AMBE3K_AMBE_BYTES);
    return 0;
}

int ambe3k_decode(ambe3k_t *h, const uint8_t ambe9[AMBE3K_AMBE_BYTES],
                  int16_t pcm[AMBE3K_PCM_SAMPLES])
{
    uint8_t pkt[sizeof HDR_CHANNEL + AMBE3K_AMBE_BYTES];
    memcpy(pkt, HDR_CHANNEL, sizeof HDR_CHANNEL);
    memcpy(pkt + sizeof HDR_CHANNEL, ambe9, AMBE3K_AMBE_BYTES);
    uint8_t pl[RX_BUF_MAX]; size_t pn;
    int rc = transact(h->fd, pkt, sizeof pkt, DV_TYPE_SPEECH, pl, sizeof pl, &pn, CODEC_TIMEOUT_MS);
    if (rc < 0) return rc;
    /* SPEECH payload = [0x00][0xA0][320 bytes] */
    if (pn < 2 + AMBE3K_PCM_SAMPLES * 2) { (void)tcflush(h->fd, TCIFLUSH); return AMBE3K_EPROTO; }
    const uint8_t *q = pl + 2;
    for (int i = 0; i < AMBE3K_PCM_SAMPLES; i++, q += 2)  /* big-endian -> host */
        pcm[i] = (int16_t)((uint16_t)q[0] << 8 | q[1]);
    return 0;
}
