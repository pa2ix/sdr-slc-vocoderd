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

/* ambe3k_sim — a simulated AMBE-3000 on a pseudo-terminal.
 *
 * Speaks the DVSI packet protocol the daemon's driver uses (product id,
 * RATEP, GAIN, CHANNEL decode, SPEECH encode) with mbelib-neo standing in for
 * the silicon, so the whole daemon can be exercised without a DVstick: run
 *
 *     ./ambe3k_sim /tmp/ambe &
 *     ./sdr-slc-vocoderd --hw-dev /tmp/ambe
 *
 * The path given is a symlink to the pty, so it stays valid across a
 * simulated unplug/replug. Faults are injected with one-word commands on
 * stdin:
 *     late [MS] delay the next reply by MS ms (default 150: past the driver's
 *              80 ms codec timeout)
 *     drop     swallow the next reply entirely
 *     unplug   close the pty (the driver sees the device vanish)
 *     replug   create a new pty and repoint the symlink
 *     stats    print transaction counts
 *     quit
 * It is a test tool: its audio is mbelib-neo's, not DVSI's. */
#define _XOPEN_SOURCE 600
#define _DEFAULT_SOURCE
#include "frames.h"
#include "swvoc.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static int master = -1, slave_hold = -1;
static const char *link_path;
static int rate_dstar = 0;
static int late_next = 0, drop_next = 0;   /* late_next: delay in ms */
static unsigned long n_dec, n_enc, n_rate, n_other;
static swdec_t *dec2450, *dec2400;
static swenc_t *enc2450, *enc2400;

static int make_pty(void)
{
    master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) < 0 || unlockpt(master) < 0) { perror("pty"); return -1; }
    const char *name = ptsname(master);
    /* Hold the slave open ourselves (raw), so the master never reads EIO
     * between the daemon's opens. */
    slave_hold = open(name, O_RDWR | O_NOCTTY);
    struct termios t;
    if (slave_hold >= 0 && tcgetattr(slave_hold, &t) == 0) { cfmakeraw(&t); tcsetattr(slave_hold, TCSANOW, &t); }
    unlink(link_path);
    if (symlink(name, link_path) < 0) { perror("symlink"); return -1; }
    fprintf(stderr, "sim: AMBE-3000 on %s (-> %s)\n", link_path, name);
    return 0;
}

static void unplug(void)
{
    if (master >= 0) close(master);
    if (slave_hold >= 0) close(slave_hold);
    master = slave_hold = -1;
    unlink(link_path);
    fprintf(stderr, "sim: unplugged\n");
}

static void reply(const uint8_t *pkt, size_t n)
{
    if (drop_next) { drop_next = 0; fprintf(stderr, "sim: reply dropped\n"); return; }
    if (late_next) {
        struct timespec ts = { late_next / 1000, (late_next % 1000) * 1000000L };
        late_next = 0;
        nanosleep(&ts, NULL);
        fprintf(stderr, "sim: reply sent late\n");
    }
    if (master >= 0 && write(master, pkt, n) < 0) { /* unplugged meanwhile */ }
}

static void control_reply(uint8_t field, const uint8_t *extra, size_t n)
{
    uint8_t pkt[64];
    size_t len = 1 + n;
    pkt[0] = 0x61; pkt[1] = (uint8_t)(len >> 8); pkt[2] = (uint8_t)len; pkt[3] = 0x00; pkt[4] = field;
    memcpy(pkt + 5, extra, n);
    reply(pkt, 5 + n);
}

static void handle(uint8_t type, const uint8_t *pl, size_t n)
{
    if (type == 0x00 && n >= 1) {
        if (pl[0] == 0x30) {                    /* product id */
            static const char id[] = "AMBE3000SIM";
            control_reply(0x30, (const uint8_t *)id, sizeof id);
            n_other++;
        } else if (pl[0] == 0x0A && n >= 13) {  /* RATEP: D-STAR starts 01 30, DMR 04 31 */
            rate_dstar = (pl[1] == 0x01 && pl[2] == 0x30);
            swdec_reset(dec2450); swdec_reset(dec2400); swenc_reset(enc2450); swenc_reset(enc2400);
            uint8_t ok = 0; control_reply(0x0A, &ok, 1);
            n_rate++;
        } else if (pl[0] == 0x4B) {             /* gain: accepted, not modelled */
            uint8_t ok = 0; control_reply(0x4B, &ok, 1);
            n_other++;
        }
        return;
    }
    if (type == 0x01 && n >= 2 + 9) {           /* CHANNEL -> decode -> SPEECH */
        char fr[4][24]; int16_t pcm[160];
        if (rate_dstar) { frames_dstar_to_fr(pl + 2, fr); swdec_frame(dec2400, (const char (*)[24])fr, pcm, NULL); }
        else            { frames_dmr_air_to_fr(pl + 2, fr); swdec_frame(dec2450, (const char (*)[24])fr, pcm, NULL); }
        uint8_t pkt[6 + 320] = { 0x61, 0x01, 0x42, 0x02, 0x00, 0xA0 };
        for (int i = 0; i < 160; i++) { pkt[6 + 2 * i] = (uint8_t)((uint16_t)pcm[i] >> 8); pkt[7 + 2 * i] = (uint8_t)pcm[i]; }
        reply(pkt, sizeof pkt);
        n_dec++;
        return;
    }
    if (type == 0x02 && n >= 2 + 320) {         /* SPEECH -> encode -> CHANNEL */
        int16_t pcm[160]; char fr[4][24];
        for (int i = 0; i < 160; i++) pcm[i] = (int16_t)((uint16_t)pl[2 + 2 * i] << 8 | pl[3 + 2 * i]);
        uint8_t pkt[6 + 9] = { 0x61, 0x00, 0x0B, 0x01, 0x01, 0x48 };
        if (rate_dstar) { swenc_frame(enc2400, pcm, fr); frames_fr_to_dstar((const char (*)[24])fr, pkt + 6); }
        else            { swenc_frame(enc2450, pcm, fr); frames_fr_to_dmr_air((const char (*)[24])fr, pkt + 6); }
        reply(pkt, sizeof pkt);
        n_enc++;
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s LINK_PATH   (then point the daemon's --hw-dev at LINK_PATH)\n", argv[0]); return 2; }
    link_path = argv[1];
    dec2450 = swdec_create(SWVOC_AMBE2450); dec2400 = swdec_create(SWVOC_AMBE2400);
    enc2450 = swenc_create(SWVOC_AMBE2450); enc2400 = swenc_create(SWVOC_AMBE2400);
    if (make_pty() < 0) return 1;
    setvbuf(stdout, NULL, _IOLBF, 0);

    uint8_t buf[4096]; size_t have = 0;
    for (;;) {
        struct pollfd p[2] = { { 0, POLLIN, 0 }, { master, POLLIN, 0 } };
        int np = master >= 0 ? 2 : 1;
        if (poll(p, (nfds_t)np, 200) < 0) { if (errno == EINTR) continue; break; }
        if (p[0].revents & (POLLIN | POLLHUP)) {
            char line[64];
            if (!fgets(line, sizeof line, stdin)) break;
            if (!strncmp(line, "late", 4)) { late_next = atoi(line + 4); if (late_next <= 0) late_next = 150; }
            else if (!strncmp(line, "drop", 4)) drop_next = 1;
            else if (!strncmp(line, "unplug", 6)) unplug();
            else if (!strncmp(line, "replug", 6)) { if (master < 0) make_pty(); have = 0; }
            else if (!strncmp(line, "stats", 5)) printf("stats decode=%lu encode=%lu ratep=%lu other=%lu\n", n_dec, n_enc, n_rate, n_other);
            else if (!strncmp(line, "quit", 4)) break;
        }
        if (np == 2 && (p[1].revents & POLLIN)) {
            ssize_t r = read(master, buf + have, sizeof buf - have);
            if (r <= 0) continue;
            have += (size_t)r;
            for (;;) {                              /* parse complete packets */
                size_t i = 0;
                while (i < have && buf[i] != 0x61) i++;
                if (i) { memmove(buf, buf + i, have - i); have -= i; }
                if (have < 4) break;
                size_t len = (size_t)buf[1] << 8 | buf[2];
                if (have < 4 + len) break;
                handle(buf[3], buf + 4, len);
                memmove(buf, buf + 4 + len, have - 4 - len);
                have -= 4 + len;
            }
        }
    }
    unplug();
    return 0;
}
