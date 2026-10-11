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

/* ambe3k_loopback.c - standalone bench harness for the ambe3k module.
 *
 * Exercises the DVstick30 / AMBE-3000 on its own, without the daemon.
 *
 * Modes:
 *   --find                 discover the stick, print device + product id, exit
 *   --tone [HZ] [SECS]     synth a sine, run PCM->AMBE->PCM, analyse (default 1000 2)
 *   --in F --out G         file loopback: read PCM, encode+decode, write PCM
 *   --encode-only          --in PCM  -> --out raw 9-byte AMBE frames
 *   --decode-only          --in AMBE -> --out PCM
 *   --ambe-out F           during loopback, also dump the AMBE frames to F
 *   --dev PATH             skip discovery, use PATH
 *   --dstar                configure the chip for D-STAR (2400x1200) instead
 *                          of DMR before running any of the above; the frame
 *                          is still 9 bytes. Verifies the RATEP is accepted
 *                          and that a tone survives the D-STAR rate.
 *
 * PCM files are raw signed 16-bit LITTLE-endian, mono, 8000 Hz (headerless):
 *   make a 1 kHz test tone:   sox -n -r 8000 -c 1 -e signed -b 16 t.s16 synth 2 sine 1000
 *   listen to a result:       sox -r 8000 -c 1 -e signed -b 16 out.s16 out.wav
 *   feed your bench kit:       (its pcm tools already speak this format)
 *
 * The tone analyser reports the dominant output frequency. This is the exact
 * check that caught the software encoder's bit-order bug historically (1 kHz
 * in -> ~575 Hz out). If the chip's 9-byte frame order matches the daemon's
 * on-air wire order, a 1 kHz tone comes back dominated by ~1 kHz.
 */
#define _POSIX_C_SOURCE 200809L
#include "ambe3k.h"
#include "frames.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SR   8000
#define N    AMBE3K_PCM_SAMPLES   /* 160 */
#define NB   AMBE3K_AMBE_BYTES    /* 9   */

/* ---- little-endian PCM file I/O ------------------------------------------ */

static size_t read_pcm_frame(FILE *f, int16_t pcm[N])
{
    uint8_t b[N * 2];
    size_t got = fread(b, 1, sizeof b, f);
    size_t samples = got / 2;
    for (size_t i = 0; i < samples; i++)
        pcm[i] = (int16_t)((uint16_t)b[2*i] | (uint16_t)b[2*i+1] << 8);
    for (size_t i = samples; i < N; i++) pcm[i] = 0;  /* zero-pad last frame */
    return samples;
}

static void write_pcm_frame(FILE *f, const int16_t pcm[N])
{
    uint8_t b[N * 2];
    for (int i = 0; i < N; i++) {
        uint16_t v = (uint16_t)pcm[i];
        b[2*i]   = (uint8_t)(v & 0xFF);
        b[2*i+1] = (uint8_t)(v >> 8);
    }
    fwrite(b, 1, sizeof b, f);
}

static void hexdump9(const char *tag, const uint8_t *p)
{
    fprintf(stderr, "%s", tag);
    for (int i = 0; i < NB; i++) fprintf(stderr, " %02x", p[i]);
    fprintf(stderr, "\n");
}

/* Load AMBE frames from either a raw binary file (9-byte frames back to back)
 * or a hex tap like "0.0000 F a82b0364ed5d51c10a" (last 18-hex token per line;
 * F/T markers and timestamps ignored). Returns malloc'd array of *count frames
 * (9 bytes each) or NULL. If deint, each frame is passed through
 * frames_dmr_air_to_canonical (on-air -> canonical) as it is loaded. */
static uint8_t *load_frames(const char *path, size_t *count, int deint)
{
    *count = 0;
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno)); return NULL; }

    /* read whole file (these are small) */
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return NULL; }
    uint8_t *raw = malloc((size_t)sz);
    if (!raw || fread(raw, 1, (size_t)sz, f) != (size_t)sz) { free(raw); fclose(f); return NULL; }
    fclose(f);

    /* sniff: any NUL / high bit in the first chunk => treat as binary */
    int binary = 0;
    for (long i = 0; i < sz && i < 512; i++) if (raw[i] == 0 || raw[i] > 127) { binary = 1; break; }

    size_t cap = 256, n = 0;
    uint8_t *out = malloc(cap * NB);

    if (binary) {
        n = (size_t)sz / NB;
        if (n > cap) { cap = n; out = realloc(out, cap * NB); }
        memcpy(out, raw, n * NB);
    } else {
        /* line-oriented: last whitespace token of exactly 18 hex chars */
        char *p = (char *)raw, *end = (char *)raw + sz;
        while (p < end) {
            char *nl = memchr(p, '\n', (size_t)(end - p));
            char *line_end = nl ? nl : end;
            /* find last token on the line */
            char *tok = NULL, *q = p;
            while (q < line_end) {
                while (q < line_end && (*q == ' ' || *q == '\t' || *q == '\r')) q++;
                if (q >= line_end) break;
                char *ts = q;
                while (q < line_end && *q != ' ' && *q != '\t' && *q != '\r') q++;
                tok = ts;   /* remember last token start; length = q - ts */
                if (q - ts == 18) {
                    int ok = 1; uint8_t fr[NB];
                    for (int b = 0; b < NB; b++) {
                        int hi = ts[2*b], lo = ts[2*b+1];
                        #define HEXV(c) ((c)>='0'&&(c)<='9'?(c)-'0':((c)|32)>='a'&&((c)|32)<='f'?((c)|32)-'a'+10:-1)
                        int h = HEXV(hi), l = HEXV(lo);
                        #undef HEXV
                        if (h < 0 || l < 0) { ok = 0; break; }
                        fr[b] = (uint8_t)(h << 4 | l);
                    }
                    if (ok) {
                        if (n == cap) { cap *= 2; out = realloc(out, cap * NB); }
                        memcpy(out + n * NB, fr, NB);
                        n++;
                    }
                }
            }
            (void)tok;
            p = nl ? nl + 1 : end;
        }
    }
    free(raw);

    if (deint) {
        for (size_t i = 0; i < n; i++) {
            uint8_t tmp[NB];
            frames_dmr_air_to_canonical(out + i * NB, tmp);
            memcpy(out + i * NB, tmp, NB);
        }
    }
    *count = n;
    return out;
}

/* ---- Goertzel dominant-frequency scan ------------------------------------ */

static double goertzel(const int16_t *x, size_t n, double freq)
{
    double w = 2.0 * M_PI * freq / SR;
    double c = 2.0 * cos(w), s1 = 0, s2 = 0;
    for (size_t i = 0; i < n; i++) {
        double s0 = x[i] + c * s1 - s2;
        s2 = s1; s1 = s0;
    }
    return s1*s1 + s2*s2 - c*s1*s2;   /* power at freq */
}

static void analyse_tone(const int16_t *x, size_t n, double f_in)
{
    /* skip vocoder warm-up (~first 20 frames) */
    size_t skip = 20 * N;
    if (skip + N >= n) skip = 0;
    const int16_t *y = x + skip;
    size_t m = n - skip;

    double best_f = 0, best_p = 0, total = 0;
    for (double f = 100; f <= 3800; f += 12.5) {
        double p = goertzel(y, m, f);
        total += p;
        if (p > best_p) { best_p = p; best_f = f; }
    }
    double p_in = goertzel(y, m, f_in);

    double rms = 0;
    for (size_t i = 0; i < m; i++) rms += (double)y[i]*y[i];
    rms = sqrt(rms / m);

    fprintf(stderr, "\n--- tone analysis (%.0f Hz in) ---\n", f_in);
    fprintf(stderr, "output RMS            : %.0f  (%.1f dBFS)\n",
            rms, 20*log10((rms + 1e-9)/32768.0));
    fprintf(stderr, "dominant output freq  : %.0f Hz\n", best_f);
    fprintf(stderr, "energy at input freq  : %.1f %% of scanned band\n",
            100.0 * p_in / (total + 1e-9));
    double err = fabs(best_f - f_in);
    if (err <= 62.5)
        fprintf(stderr, "VERDICT: PASS - output tracks input within one bin\n");
    else
        fprintf(stderr, "VERDICT: CHECK - dominant freq is %.0f Hz off; suspect frame/bit order\n", err);
    fprintf(stderr, "(the daemon's Python bench kit remains the authority: "
                    "pipe --out through it for harmonicity/pitch.)\n");
}

/* ---- device selection ---------------------------------------------------- */

static int g_set_gain = 0, g_gain_in = 0, g_gain_out = 0;

static int g_dstar = 0;
static ambe3k_t *open_opened(ambe3k_t *h)
{
    if (h && g_dstar) {
        if (ambe3k_set_rate(h, AMBE3K_RATE_DSTAR) < 0) {
            fprintf(stderr, "chip refused the D-STAR rate\n");
            ambe3k_close(h); return NULL;
        }
        fprintf(stderr, "chip configured for %s\n", ambe3k_rate_name(ambe3k_rate(h)));
    }
    if (h && g_set_gain) {
        if (ambe3k_set_gain(h, g_gain_in, g_gain_out) == 0)
            fprintf(stderr, "gain set: in %+d dB, out %+d dB\n", g_gain_in, g_gain_out);
        else
            fprintf(stderr, "warning: set_gain failed\n");
    }
    return h;
}

static ambe3k_t *open_device(const char *dev_opt)
{
    if (dev_opt) return open_opened(ambe3k_open(dev_opt));

    char path[256] = {0}, prodid[AMBE3K_PRODID_MAX] = {0}, hint[256] = {0};
    switch (ambe3k_find(path, sizeof path, prodid, sizeof prodid, hint, sizeof hint)) {
    case AMBE3K_FOUND:
        fprintf(stderr, "discovered %s on %s\n", prodid, path);
        return open_opened(ambe3k_open(path));
    case AMBE3K_PERMISSION:
    case AMBE3K_NO_RESPONSE:
        fprintf(stderr, "%s\n", hint);
        return NULL;
    case AMBE3K_ABSENT:
    default:
        fprintf(stderr, "no AMBE-3000 stick found (scanned /dev/ttyUSB*).\n"
                        "plug it in, or pass --dev /dev/ttyUSBx.\n");
        return NULL;
    }
}

/* ---- main ---------------------------------------------------------------- */

static void usage(const char *a0)
{
    fprintf(stderr,
        "usage: %s [--dev PATH] [mode]\n"
        "  --find                    discover stick, print id, exit\n"
        "  --tone [HZ] [SECS]        synth sine, loopback, analyse (default 1000 2)\n"
        "  --in F --out G            PCM(s16le,8k,mono) loopback through the chip\n"
        "  --encode-only --in F --out G   PCM -> raw 9-byte AMBE frames\n"
        "  --decode-only --in F --out G   AMBE frames -> PCM (F: raw 9-byte, or\n"
        "                                 a hex tap '<ts> F <9-byte-hex>' per line)\n"
        "  --deinterleave                 with --decode-only: on-air -> canonical\n"
        "                                 each frame (on-air -> canonical) first\n"
        "  --dstar                        configure the chip for D-STAR (2400x1200)\n"
        "                                 before the test; frames stay 9 bytes\n"
        "  --gain-out N / --gain-in N      set chip decode-output / encode-input\n"
        "                                 gain in dB before running (default 0)\n"
        "  --ambe-out F              also write AMBE frames during full loopback\n",
        a0);
}

int main(int argc, char **argv)
{
    const char *dev = NULL, *in = NULL, *out = NULL, *ambe_out = NULL;
    int do_find = 0, do_tone = 0, enc_only = 0, dec_only = 0, deint = 0;
    double tone_hz = 1000.0, tone_s = 2.0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i+1 < argc) dev = argv[++i];
        else if (!strcmp(argv[i], "--find")) do_find = 1;
        else if (!strcmp(argv[i], "--tone")) {
            do_tone = 1;
            if (i+1 < argc && argv[i+1][0] != '-') tone_hz = atof(argv[++i]);
            if (i+1 < argc && argv[i+1][0] != '-') tone_s  = atof(argv[++i]);
        }
        else if (!strcmp(argv[i], "--in")  && i+1 < argc) in  = argv[++i];
        else if (!strcmp(argv[i], "--out") && i+1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--ambe-out") && i+1 < argc) ambe_out = argv[++i];
        else if (!strcmp(argv[i], "--encode-only")) enc_only = 1;
        else if (!strcmp(argv[i], "--decode-only")) dec_only = 1;
        else if (!strcmp(argv[i], "--deinterleave")) deint = 1;
        else if (!strcmp(argv[i], "--dstar")) g_dstar = 1;
        else if (!strcmp(argv[i], "--gain-out") && i+1 < argc) { g_gain_out = atoi(argv[++i]); g_set_gain = 1; }
        else if (!strcmp(argv[i], "--gain-in")  && i+1 < argc) { g_gain_in  = atoi(argv[++i]); g_set_gain = 1; }
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(argv[0]); return 0; }
        else { fprintf(stderr, "unknown arg: %s\n", argv[i]); usage(argv[0]); return 2; }
    }

    if (do_find) {
        char path[256] = {0}, prodid[AMBE3K_PRODID_MAX] = {0}, hint[256] = {0};
        switch (ambe3k_find(path, sizeof path, prodid, sizeof prodid, hint, sizeof hint)) {
        case AMBE3K_FOUND:       printf("%s\t%s\n", path, prodid); return 0;
        case AMBE3K_PERMISSION:  fprintf(stderr, "%s\n", hint);    return 3;
        case AMBE3K_NO_RESPONSE: fprintf(stderr, "%s\n", hint);    return 4;
        case AMBE3K_ABSENT:
        default:                 fprintf(stderr, "no AMBE-3000 stick found.\n"); return 1;
        }
    }

    if (!do_tone && !in && !dec_only) do_tone = 1;   /* default to the self-test */

    /* ---- decode-only: pre-load frames first (validates the file even with no
     * hardware) then decode them through the chip -------------------------- */
    if (dec_only) {
        if (!in) { fprintf(stderr, "--decode-only needs --in FILE\n"); return 2; }
        size_t nf = 0;
        uint8_t *frames = load_frames(in, &nf, deint);
        if (!frames || nf == 0) {
            fprintf(stderr, "no AMBE frames read from %s\n", in);
            free(frames); return 1;
        }
        fprintf(stderr, "loaded %zu AMBE frame%s from %s (%s)\n",
                nf, nf == 1 ? "" : "s", in,
                deint ? "deinterleaved -> canonical order"
                      : "as-is, on-air order");

        ambe3k_t *h = open_device(dev);
        if (!h) { free(frames); return 1; }

        FILE *fo = NULL;
        if (out) {
            fo = strcmp(out, "-") ? fopen(out, "wb") : stdout;
            if (!fo) { fprintf(stderr, "cannot open %s\n", out); ambe3k_close(h); free(frames); return 1; }
        }
        unsigned long ok = 0, fail = 0; int shown = 0;
        double sumsq = 0; size_t nsamp = 0;
        for (size_t i = 0; i < nf; i++) {
            if (!shown) { hexdump9(deint ? "first frame (deint):" : "first frame (as-is):",
                                   frames + i * NB); shown = 1; }
            int16_t pcm[N];
            if (ambe3k_decode(h, frames + i * NB, pcm) < 0) { fail++; memset(pcm, 0, sizeof pcm); }
            else ok++;
            for (int k = 0; k < N; k++) { sumsq += (double)pcm[k]*pcm[k]; nsamp++; }
            if (fo) write_pcm_frame(fo, pcm);
        }
        double rms = nsamp ? sqrt(sumsq / (double)nsamp) : 0;
        fprintf(stderr, "frames ok=%lu fail=%lu  decoded RMS %.0f (%.1f dBFS)%s%s\n",
                ok, fail, rms, 20*log10((rms + 1e-9)/32768.0),
                out ? " -> " : "", out ? out : "");
        if (fo && fo != stdout) fclose(fo);
        ambe3k_close(h);
        free(frames);
        return fail ? 1 : 0;
    }

    ambe3k_t *h = open_device(dev);
    if (!h) return 1;

    int rc = 0;
    unsigned long ok = 0, fail = 0;
    int shown = 0;

    if (do_tone) {
        size_t total = (size_t)(tone_s * SR);
        total -= total % N;
        int16_t *sig = malloc(total * sizeof *sig);
        if (!sig) { ambe3k_close(h); return 1; }
        for (size_t f = 0; f < total; f += N) {
            int16_t pcm[N], back[N]; uint8_t ambe[NB];
            for (int i = 0; i < N; i++) {
                double t = (double)(f + i) / SR;
                double ramp = (f + i) < SR/20 ? (double)(f+i)/(SR/20) : 1.0;  /* 50 ms fade-in */
                pcm[i] = (int16_t)lround(ramp * 12000.0 * sin(2*M_PI*tone_hz*t));
            }
            if (ambe3k_encode(h, pcm, ambe) < 0) { fail++; memset(back, 0, sizeof back); }
            else {
                if (!shown) { hexdump9("first AMBE frame:", ambe); shown = 1; }
                if (ambe3k_decode(h, ambe, back) < 0) { fail++; memset(back, 0, sizeof back); }
                else ok++;
            }
            memcpy(sig + f, back, sizeof back);
        }
        fprintf(stderr, "frames ok=%lu fail=%lu\n", ok, fail);
        if (out) { FILE *g = fopen(out, "wb"); if (g) { fwrite(sig, sizeof(int16_t), total, g); fclose(g);
                   fprintf(stderr, "wrote decoded PCM to %s\n", out); } }
        analyse_tone(sig, total, tone_hz);
        free(sig);
        ambe3k_close(h);
        return fail ? 1 : 0;
    }

    /* file modes: encode-only or full loopback (PCM streamed in) */
    FILE *fi = strcmp(in, "-") ? fopen(in, "rb") : stdin;
    if (!fi) { fprintf(stderr, "cannot open %s\n", in); ambe3k_close(h); return 1; }
    FILE *fo = NULL;
    if (out) { fo = strcmp(out, "-") ? fopen(out, "wb") : stdout;
               if (!fo) { fprintf(stderr, "cannot open %s\n", out); rc = 1; goto done; } }
    FILE *fa = ambe_out ? fopen(ambe_out, "wb") : NULL;

    {   /* encode-only or full loopback */
        int16_t pcm[N];
        while (read_pcm_frame(fi, pcm) > 0) {
            uint8_t ambe[NB];
            if (ambe3k_encode(h, pcm, ambe) < 0) { fail++; continue; }
            if (!shown) { hexdump9("first AMBE frame:", ambe); shown = 1; }
            if (fa) fwrite(ambe, 1, NB, fa);
            if (enc_only) { if (fo) fwrite(ambe, 1, NB, fo); ok++; continue; }
            int16_t back[N];
            if (ambe3k_decode(h, ambe, back) < 0) { fail++; memset(back, 0, sizeof back); }
            else ok++;
            if (fo) write_pcm_frame(fo, back);
        }
    }
    fprintf(stderr, "frames ok=%lu fail=%lu\n", ok, fail);
    if (fa) fclose(fa);

done:
    if (fi && fi != stdin) fclose(fi);
    if (fo && fo != stdout) fclose(fo);
    ambe3k_close(h);
    return rc ? rc : (fail ? 1 : 0);
}
