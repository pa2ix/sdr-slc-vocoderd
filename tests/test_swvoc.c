/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * sdr-slc-vocoderd — digital voice vocoder service for the SDR-SLC protocol suite.
 * Copyright (C) 2026 Ivo van Ling (PA2IX)
 *
 * test_swvoc — the mbelib-neo backend through the real wire formats.
 *
 *  1. Tone loopback per format (PCM -> encode -> wire bytes -> decode -> PCM):
 *     the dominant output frequency must track the input, and the level must
 *     come back within a few dB. This is the check that caught OpenDMR's
 *     parameter-packing bug in sdr-slc-codecd (1 kHz in -> 575 Hz out).
 *  2. Classification: clean frames are VOICE; random bytes (an idle slot, or
 *     noise with the squelch open) must almost never look like voice.
 *  3. Erasures conceal (repeat) and then mute, per TIA-102.BABA-1.
 */
#define _DEFAULT_SOURCE 1          /* M_PI */
#include "frames.h"
#include "swvoc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

static uint32_t rng = 0xC0FFEEu;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

enum { F_DMR, F_YSF, F_DSTAR };
static const char *fname[] = { "ambe-dmr", "ambe-ysf", "ambe-dstar" };

/* Encode one PCM frame to wire bytes in format f. */
static int enc_wire(int f, swenc_t *e, const int16_t pcm[160], uint8_t wire[9])
{
    char fr[4][24], d[49];
    switch (f) {
    case F_DMR:   if (swenc_frame(e, pcm, fr) < 0) return -1; frames_fr_to_dmr_air((const char (*)[24])fr, wire); return 0;
    case F_YSF:   if (swenc_params(e, pcm, d) < 0) return -1; frames_d_to_voice49(d, wire); return 0;
    default:      if (swenc_frame(e, pcm, fr) < 0) return -1; return frames_fr_to_dstar((const char (*)[24])fr, wire);
    }
}

static int dec_wire(int f, swdec_t *dd, const uint8_t wire[9], int16_t pcm[160], swvoc_info_t *inf)
{
    char fr[4][24], d[49];
    switch (f) {
    case F_DMR:   frames_dmr_air_to_fr(wire, fr); return swdec_frame(dd, (const char (*)[24])fr, pcm, inf);
    case F_YSF:   frames_voice49_to_d(wire, d); return swdec_params(dd, d, pcm, inf);
    default:      if (frames_dstar_to_fr(wire, fr) < 0) return -1; return swdec_frame(dd, (const char (*)[24])fr, pcm, inf);
    }
}

static double rms(const int16_t *x, int n) { double s = 0; for (int i = 0; i < n; i++) s += (double)x[i] * x[i]; return sqrt(s / n); }

/* Dominant frequency by a 10 Hz-step DFT scan over 200..3800 Hz. */
static double dominant_hz(const int16_t *x, int n)
{
    double best = 0, bf = 0;
    for (double f = 200; f <= 3800; f += 10) {
        double re = 0, im = 0, w = 2 * M_PI * f / 8000.0;
        for (int i = 0; i < n; i++) { re += x[i] * cos(w * i); im += x[i] * sin(w * i); }
        double p = re * re + im * im;
        if (p > best) { best = p; bf = f; }
    }
    return bf;
}

static void tone_loopback(int f, swvoc_family_t fam, double hz, double dbfs)
{
    swenc_t *e = swenc_create(fam); swdec_t *d = swdec_create(fam);
    enum { FRAMES = 75 };            /* 1.5 s */
    static int16_t out[FRAMES * 160];
    double amp = 32767.0 * pow(10, dbfs / 20.0);
    int voice = 0;
    for (int k = 0; k < FRAMES; k++) {
        int16_t in[160]; uint8_t w[9]; swvoc_info_t inf;
        for (int i = 0; i < 160; i++) in[i] = (int16_t)lrint(amp * sin(2 * M_PI * hz * (k * 160 + i) / 8000.0));
        CHECK(enc_wire(f, e, in, w) == 0, "%s encode", fname[f]);
        CHECK(dec_wire(f, d, w, out + k * 160, &inf) == 0, "%s decode", fname[f]);
        if (inf.cls == SWVOC_VOICE || inf.cls == SWVOC_TONE) voice++;
    }
    const int skip = 25 * 160;      /* let analysis and prediction settle */
    double got = dominant_hz(out + skip, FRAMES * 160 - skip);
    double lvl = 20 * log10(rms(out + skip, FRAMES * 160 - skip) / 32767.0) + 3.01; /* sine: peak dB = rms dB + 3 */
    printf("  %-10s %4.0f Hz @ %5.1f dBFS -> %4.0f Hz @ %5.1f dBFS, %d/%d frames voice/tone\n",
           fname[f], hz, dbfs, got, lvl, voice, FRAMES);
    CHECK(fabs(got - hz) <= 30, "%s: %g Hz in, %g Hz out", fname[f], hz, got);
    CHECK(fabs(lvl - dbfs) <= 4.0, "%s: level %g dBFS in, %g dBFS out", fname[f], dbfs, lvl);
    CHECK(voice == FRAMES, "%s: only %d/%d frames classified voice/tone", fname[f], voice, FRAMES);
    swenc_destroy(e); swdec_destroy(d);
}

static void noise_classification(int f, swvoc_family_t fam)
{
    swdec_t *d = swdec_create(fam);
    int voice = 0, claims = 0;
    swvoc_window_t win; swvoc_window_reset(&win);
    const int N = 20000;
    for (int k = 0; k < N; k++) {
        uint8_t w[9]; int16_t pcm[160]; swvoc_info_t inf;
        for (int i = 0; i < 9; i++) w[i] = (uint8_t)rnd();
        if (f == F_YSF) w[6] &= 0x80;
        dec_wire(f, d, w, pcm, &inf);
        if (swvoc_is_activity(inf.cls)) voice++;
        swvoc_window_push(&win, &inf);
        if (swvoc_window_ok(&win, SWVOC_WIN_CLAIM)) claims++;
    }
    printf("  %-10s random frames: %4.1f%% decode as voice frames, %d window claims\n", fname[f], 100.0 * voice / N, claims);
    /* Single random frames DO decode as voice (Golay maps almost any word to
     * a codeword), which is why the arbiter uses the error window instead.
     * YSF carries no AMBE FEC, so random bits cannot be told from voice at
     * this layer at all: the client's VCH decoding is what rejects them. */
    if (f != F_YSF) CHECK(claims == 0, "%s: random frames claimed the chip %d times", fname[f], claims);
    swdec_destroy(d);
}

static void erasures(swvoc_family_t fam, const char *name)
{
    swdec_t *d = swdec_create(fam); swenc_t *e = swenc_create(fam);
    int16_t pcm[160], in[160]; char fr[4][24]; swvoc_info_t inf;
    for (int k = 0; k < 20; k++) {
        for (int i = 0; i < 160; i++) in[i] = (int16_t)lrint(3000 * sin(2 * M_PI * 400 * (k * 160 + i) / 8000.0));
        swenc_frame(e, in, fr); swdec_frame(d, (const char (*)[24])fr, pcm, &inf);
    }
    swvoc_class_t c[6];
    for (int k = 0; k < 6; k++) { CHECK(swdec_erasure(d, pcm, &inf) == 0, "erasure"); c[k] = inf.cls; }
    printf("  %-10s erasures -> %s %s %s %s %s %s\n", name,
           c[0] == SWVOC_CONCEALED ? "R" : c[0] == SWVOC_MUTED ? "M" : "?", c[1] == SWVOC_CONCEALED ? "R" : c[1] == SWVOC_MUTED ? "M" : "?",
           c[2] == SWVOC_CONCEALED ? "R" : c[2] == SWVOC_MUTED ? "M" : "?", c[3] == SWVOC_CONCEALED ? "R" : c[3] == SWVOC_MUTED ? "M" : "?",
           c[4] == SWVOC_CONCEALED ? "R" : c[4] == SWVOC_MUTED ? "M" : "?", c[5] == SWVOC_CONCEALED ? "R" : c[5] == SWVOC_MUTED ? "M" : "?");
    CHECK(c[0] == SWVOC_CONCEALED, "%s: first erasure not concealed", name);
    CHECK(c[5] == SWVOC_MUTED, "%s: six erasures in a row not muted", name);
    swdec_destroy(d); swenc_destroy(e);
}

int main(void)
{
    puts("tone loopback (encode -> wire -> decode):");
    const double tones[] = { 300, 1000, 2000 };
    for (int t = 0; t < 3; t++) {
        tone_loopback(F_DMR,   SWVOC_AMBE2450, tones[t], -20);
        tone_loopback(F_YSF,   SWVOC_AMBE2450, tones[t], -20);
        tone_loopback(F_DSTAR, SWVOC_AMBE2400, tones[t], -20);
    }
    tone_loopback(F_DMR, SWVOC_AMBE2450, 1000, -40);
    tone_loopback(F_DSTAR, SWVOC_AMBE2400, 1000, -40);
    puts("classification of random frames:");
    noise_classification(F_DMR, SWVOC_AMBE2450);
    noise_classification(F_YSF, SWVOC_AMBE2450);
    noise_classification(F_DSTAR, SWVOC_AMBE2400);
    puts("erasure concealment (R = repeat, M = mute):");
    erasures(SWVOC_AMBE2450, "AMBE+2");
    erasures(SWVOC_AMBE2400, "D-STAR");
    if (fails) { fprintf(stderr, "test_swvoc: %u failure(s)\n", fails); return 1; }
    puts("test_swvoc OK");
    return 0;
}
