/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * sdr-slc-vocoderd — digital voice vocoder service for the SDR-SLC protocol suite.
 * Copyright (C) 2026 Ivo van Ling (PA2IX)
 *
 * test_arbiter — the chip-sharing policy against a simulated AMBE-3000.
 *
 * The simulated chip is a real vocoder (its own mbelib-neo state), installed
 * through voc_set_chip_ops(). It records every transaction, so the tests can
 * check the one invariant that protects audio quality: the chip's single
 * vocoder context never passes from one stream to another without a reset.
 *
 * Scenarios are the situations seen with sdr-slc-codecd on air, plus faults:
 *   A  armed transmit sending silence + a receive slice with a real over:
 *      the receiver must get the chip (codecd: chip=3 of 8699 frames).
 *   B  a receive slice carrying noise never claims; a real over that starts
 *      mid-stream claims after warm-up, with a single crossfade frame.
 *   C  transmit speech preempts a receiving owner at once; the receiver keeps
 *      decoding in software without a single failed frame; after the transmit
 *      goes quiet the receiver takes the chip back.
 *   D  two receivers: the first keeps the chip for its over, the second gets
 *      it when the first ends.
 *   E  the chip vanishes mid-over: no frame is lost; when it returns it is
 *      picked up and used again. Repeated timeouts trigger a reopen.
 *   F  D-STAR switches the chip's rate.
 *   G  multithreaded stress: 6 streams for 3 s; invariant and no deadlock.
 */
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE 1          /* M_PI */
#include "frames.h"
#include "swvoc.h"
#include "vocoder.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static unsigned fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

/* ---- the simulated chip -------------------------------------------------- */
static pthread_mutex_t sim_lock = PTHREAD_MUTEX_INITIALIZER;
static int sim_present = 1;         /* is a device plugged in            */
static int sim_fail_mode = 0;       /* 0 ok, 1 gone (-2), 2 timeout (-1)  */
static int sim_opens = 0, sim_resets = 0, sim_xacts = 0, sim_violations = 0;
static int sim_rate = -1;
static const void *sim_last_user = NULL;   /* stream whose state the chip holds */
static int sim_reset_pending = 0;          /* a reset happened since last xact */
static swdec_t *sim_dec2450, *sim_dec2400;
static swenc_t *sim_enc2450, *sim_enc2400;
static __thread const void *tls_stream;    /* set by the test before each call */

static int sim_handle;
static void *sim_open_any(char *name, unsigned long n)
{
    pthread_mutex_lock(&sim_lock);
    void *h = sim_present ? (void *)&sim_handle : NULL;
    if (h) { sim_opens++; sim_fail_mode = 0; sim_rate = -1; sim_last_user = NULL; snprintf(name, n, "SIM-AMBE3000 on /dev/sim"); }
    pthread_mutex_unlock(&sim_lock);
    return h;
}
static void sim_close(void *h) { (void)h; }
static int sim_fail(void) { return sim_fail_mode == 1 ? -2 : sim_fail_mode == 2 ? -1 : 0; }
static int sim_set_rate(void *h, int dstar)
{
    (void)h;
    pthread_mutex_lock(&sim_lock);
    int rc = sim_fail();
    if (!rc) {
        sim_rate = dstar; sim_resets++; sim_reset_pending = 1;
        swdec_reset(sim_dec2450); swdec_reset(sim_dec2400); swenc_reset(sim_enc2450); swenc_reset(sim_enc2400);
    }
    pthread_mutex_unlock(&sim_lock);
    return rc;
}
static void sim_note_user(void)
{
    if (sim_last_user && sim_last_user != tls_stream && !sim_reset_pending) sim_violations++;
    sim_last_user = tls_stream;
    sim_reset_pending = 0;
    sim_xacts++;
}
static int sim_decode(void *h, const uint8_t in9[9], int16_t pcm[160])
{
    (void)h;
    pthread_mutex_lock(&sim_lock);
    int rc = sim_fail();
    if (!rc) {
        sim_note_user();
        char fr[4][24];
        if (sim_rate == 1) { frames_dstar_to_fr(in9, fr); swdec_frame(sim_dec2400, (const char (*)[24])fr, pcm, NULL); }
        else               { frames_dmr_air_to_fr(in9, fr); swdec_frame(sim_dec2450, (const char (*)[24])fr, pcm, NULL); }
    }
    pthread_mutex_unlock(&sim_lock);
    return rc;
}
static int sim_encode(void *h, const int16_t pcm[160], uint8_t out9[9])
{
    (void)h;
    pthread_mutex_lock(&sim_lock);
    int rc = sim_fail();
    if (!rc) {
        sim_note_user();
        char fr[4][24];
        if (sim_rate == 1) { swenc_frame(sim_enc2400, pcm, fr); frames_fr_to_dstar((const char (*)[24])fr, out9); }
        else               { swenc_frame(sim_enc2450, pcm, fr); frames_fr_to_dmr_air((const char (*)[24])fr, out9); }
    }
    pthread_mutex_unlock(&sim_lock);
    return rc;
}
static int sim_changed_flag = 0;
static int sim_changed(void) { pthread_mutex_lock(&sim_lock); int c = sim_changed_flag; sim_changed_flag = 0; pthread_mutex_unlock(&sim_lock); return c; }

static int sim_get_opens(void) { pthread_mutex_lock(&sim_lock); int n = sim_opens; pthread_mutex_unlock(&sim_lock); return n; }

static void sim_set(int present, int fail_mode)
{
    pthread_mutex_lock(&sim_lock);
    if (present != sim_present) sim_changed_flag = 1;
    sim_present = present; sim_fail_mode = fail_mode;
    pthread_mutex_unlock(&sim_lock);
}

/* ---- signal sources ------------------------------------------------------ */
static uint32_t rng = 0x5EED;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* A speech-like voiced signal (gliding pitch, three formant-ish partials,
 * syllable envelope) - enough for the encoder to produce voice frames. */
static void speech(int16_t pcm[160], long frame, double amp)
{
    for (int i = 0; i < 160; i++) {
        double t = (frame * 160 + i) / 8000.0, f0 = 120 + 30 * sin(2 * M_PI * 0.7 * t);
        double v = sin(2 * M_PI * f0 * t) + 0.6 * sin(2 * M_PI * 2.3 * f0 * t) + 0.3 * sin(2 * M_PI * 5.1 * f0 * t);
        pcm[i] = (int16_t)lrint(amp * v * (0.6 + 0.4 * sin(2 * M_PI * 3 * t)));
    }
}

/* A talker on the air: encodes speech into wire frames of a format. */
typedef struct { swenc_t *enc; voc_fmt_t fmt; long n; } talker_t;
static void talker_init(talker_t *t, voc_fmt_t fmt) { t->enc = swenc_create(fmt == VOC_FMT_DSTAR ? SWVOC_AMBE2400 : SWVOC_AMBE2450); t->fmt = fmt; t->n = 0; }
static void talker_frame(talker_t *t, uint8_t wire[9])
{
    int16_t pcm[160]; char fr[4][24];
    speech(pcm, t->n++, 6000);
    swenc_frame(t->enc, pcm, fr);
    if (t->fmt == VOC_FMT_DSTAR) frames_fr_to_dstar((const char (*)[24])fr, wire);
    else frames_fr_to_dmr_air((const char (*)[24])fr, wire);
}
static void noise_frame(uint8_t wire[9]) { for (int i = 0; i < 9; i++) wire[i] = (uint8_t)rnd(); }

static void sleep_ms(int ms) { struct timespec ts = { 0, ms * 1000000L }; nanosleep(&ts, NULL); }

/* One 'tick' of simulated time: the hang times in this test are scaled so
 * one frame period is FRAME_MS of real time. */
#define FRAME_MS 4

static voc_result_t dec(voc_stream_t *s, const uint8_t *w)
{
    int16_t pcm[160]; voc_result_t r; tls_stream = s;
    int rc = voc_decode(s, w, pcm, &r);
    CHECK(rc == 0, "decode failed");
    return r;
}
static voc_result_t enc(voc_stream_t *s, int talking, long n)
{
    int16_t pcm[160]; uint8_t w[9]; voc_result_t r; tls_stream = s;
    if (talking) speech(pcm, n, 6000); else memset(pcm, 0, sizeof pcm);
    int rc = voc_encode(s, pcm, w, &r);
    CHECK(rc == 0, "encode failed");
    return r;
}

static void wait_chip(int want)
{
    for (int i = 0; i < 400 && voc_chip_present() != want; i++) sleep_ms(5);
    CHECK(voc_chip_present() == want, "chip presence did not become %d", want);
}

/* ---- scenarios ----------------------------------------------------------- */

static void scenario_A(void)
{
    puts("A  armed TX (silence) + RX slice with a real over");
    voc_stream_t *tx = voc_stream_create(VOC_FMT_DMR, 0, "tx-armed");
    voc_stream_t *rx = voc_stream_create(VOC_FMT_DMR, 1, "rx-slot1");
    talker_t t; talker_init(&t, VOC_FMT_DMR);
    int chip_rx = 0, chip_tx = 0, n = 300;
    for (int k = 0; k < n; k++) {
        if (enc(tx, 0, k).backend != VOC_BE_SW) chip_tx++;
        uint8_t w[9]; talker_frame(&t, w);
        if (dec(rx, w).backend == VOC_BE_CHIP) chip_rx++;
        sleep_ms(FRAME_MS);
    }
    printf("   RX on chip %d/%d frames, armed TX on chip %d/%d\n", chip_rx, n, chip_tx, n);
    CHECK(chip_rx >= n - 12, "receiver should be on the chip (got %d/%d)", chip_rx, n);
    CHECK(chip_tx == 0, "silent armed transmit must not hold the chip");
    voc_stream_destroy(tx); voc_stream_destroy(rx);
}

static void scenario_B(void)
{
    puts("B  RX slice: noise, then a real over starting mid-stream");
    voc_stream_t *rx = voc_stream_create(VOC_FMT_DMR, 1, "rx-noise");
    talker_t t; talker_init(&t, VOC_FMT_DMR);
    int chip_noise = 0;
    for (int k = 0; k < 400; k++) { uint8_t w[9]; noise_frame(w); if (dec(rx, w).backend != VOC_BE_SW) chip_noise++; sleep_ms(1); }
    CHECK(chip_noise == 0, "noise claimed the chip for %d frames", chip_noise);
    char seq[64] = {0};
    for (int k = 0; k < 40; k++) {
        uint8_t w[9]; talker_frame(&t, w);
        voc_result_t r = dec(rx, w);
        if (k < 63) seq[k] = r.backend == VOC_BE_CHIP ? 'C' : r.backend == VOC_BE_XFADE ? 'X' : 's';
        sleep_ms(FRAME_MS);
    }
    printf("   noise: 0 chip frames of 400; over: %s\n", seq);
    /* Expected shape: software while the 6-frame evidence window fills (a
     * trailing noise frame that happens to be clean may count, so 5-6
     * frames), the claim frame plus 2 more silent warm-up frames, exactly
     * one crossfade frame, then the chip for the rest of the over. */
    size_t lead = strspn(seq, "s");
    CHECK(lead >= 7 && lead <= 9, "takeover after %zu software frames (expected 7-9): %s", lead, seq);
    CHECK(seq[lead] == 'X', "no single crossfade frame: %s", seq);
    CHECK(strspn(seq + lead + 1, "C") == strlen(seq + lead + 1), "not on the chip after the crossfade: %s", seq);
    voc_stream_destroy(rx);
}

static void scenario_C(void)
{
    puts("C  TX speech preempts a receiving owner; RX takes the chip back after");
    voc_stream_t *tx = voc_stream_create(VOC_FMT_DMR, 0, "tx");
    voc_stream_t *rx = voc_stream_create(VOC_FMT_DMR, 1, "rx");
    talker_t t; talker_init(&t, VOC_FMT_DMR);
    uint8_t w[9];
    for (int k = 0; k < 30; k++) { enc(tx, 0, k); talker_frame(&t, w); dec(rx, w); sleep_ms(FRAME_MS); }
    talker_frame(&t, w);
    CHECK(dec(rx, w).backend == VOC_BE_CHIP, "receiver should own the chip before the key-up");
    voc_result_t te = enc(tx, 1, 100);
    CHECK(te.backend == VOC_BE_CHIP, "transmit speech must take the chip on its first voiced frame");
    talker_frame(&t, w);
    CHECK(dec(rx, w).backend == VOC_BE_SW, "preempted receiver must continue in software");
    int tx_chip = 0;
    for (int k = 0; k < 50; k++) { if (enc(tx, 1, 101 + k).backend == VOC_BE_CHIP) tx_chip++; talker_frame(&t, w); dec(rx, w); sleep_ms(FRAME_MS); }
    CHECK(tx_chip == 50, "transmit lost the chip during its over (%d/50)", tx_chip);
    /* unkey: transmit goes silent; after tx_hang the receiver reclaims */
    int back = -1;
    for (int k = 0; k < 120; k++) {
        enc(tx, 0, k); talker_frame(&t, w);
        if (dec(rx, w).backend == VOC_BE_CHIP && back < 0) back = k;
        sleep_ms(FRAME_MS);
    }
    printf("   TX on chip 50/50 during its over; RX back on the chip %d frames after unkey\n", back);
    CHECK(back > 0 && back < 60, "receiver did not get the chip back after the transmit went quiet (%d)", back);
    voc_stream_destroy(tx); voc_stream_destroy(rx);
}

static void scenario_D(void)
{
    puts("D  two receivers with overs: first keeps it, second gets it when the first ends");
    voc_stream_t *a = voc_stream_create(VOC_FMT_DMR, 1, "rx-a");
    voc_stream_t *b = voc_stream_create(VOC_FMT_DMR, 1, "rx-b");
    talker_t ta, tb; talker_init(&ta, VOC_FMT_DMR); talker_init(&tb, VOC_FMT_DMR);
    int a_chip = 0, b_chip = 0;
    uint8_t w[9];
    for (int k = 0; k < 100; k++) {
        talker_frame(&ta, w); if (dec(a, w).backend == VOC_BE_CHIP) a_chip++;
        talker_frame(&tb, w); if (dec(b, w).backend == VOC_BE_CHIP) b_chip++;
        sleep_ms(FRAME_MS);
    }
    CHECK(a_chip > 80 && b_chip == 0, "first receiver should keep the chip (a=%d b=%d)", a_chip, b_chip);
    int b_from = -1;
    for (int k = 0; k < 100; k++) {
        noise_frame(w); dec(a, w);                      /* a's over has ended */
        talker_frame(&tb, w); if (dec(b, w).backend == VOC_BE_CHIP && b_from < 0) b_from = k;
        sleep_ms(FRAME_MS);
    }
    printf("   during both overs a=%d b=%d chip frames; b takes over %d frames after a ends\n", a_chip, b_chip, b_from);
    CHECK(b_from > 0 && b_from < 40, "second receiver did not take over (%d)", b_from);
    voc_stream_destroy(a); voc_stream_destroy(b);
}

static void scenario_E(void)
{
    puts("E  chip vanishes mid-over, comes back; repeated timeouts reopen it");
    voc_stream_t *rx = voc_stream_create(VOC_FMT_DMR, 1, "rx-e");
    talker_t t; talker_init(&t, VOC_FMT_DMR);
    uint8_t w[9];
    for (int k = 0; k < 30; k++) { talker_frame(&t, w); dec(rx, w); sleep_ms(FRAME_MS); }
    talker_frame(&t, w);
    CHECK(dec(rx, w).backend == VOC_BE_CHIP, "receiver should be on the chip");
    sim_set(0, 1);                                      /* unplugged */
    int sw = 0;
    for (int k = 0; k < 20; k++) { talker_frame(&t, w); if (dec(rx, w).backend == VOC_BE_SW) sw++; sleep_ms(FRAME_MS); }
    CHECK(sw == 20, "after unplug every frame must come from software (%d/20)", sw);
    wait_chip(0);
    int opens = sim_get_opens();
    sim_set(1, 0);                                      /* plugged back in */
    wait_chip(1);
    CHECK(sim_get_opens() == opens + 1, "chip not reopened");
    int back = -1;
    for (int k = 0; k < 40; k++) { talker_frame(&t, w); if (dec(rx, w).backend == VOC_BE_CHIP && back < 0) back = k; sleep_ms(FRAME_MS); }
    printf("   unplug: 20/20 frames in software, no failures; replug: chip again after %d frames\n", back);
    CHECK(back >= 0 && back < 10, "receiver did not return to the chip after replug (%d)", back);

    /* timeouts: three in a row make the daemon close and reopen the chip */
    opens = sim_get_opens();
    sim_set(1, 2);
    for (int k = 0; k < 5; k++) { talker_frame(&t, w); dec(rx, w); sleep_ms(FRAME_MS); }
    sim_set(1, 0);
    for (int i = 0; i < 1600 && sim_get_opens() == opens; i++) sleep_ms(5);
    wait_chip(1);
    int reopened = sim_get_opens() - opens;
    printf("   3 timeouts -> chip reopened (%d open%s)\n", reopened, reopened == 1 ? "" : "s");
    CHECK(reopened == 1, "chip not reopened after repeated timeouts");
    voc_stream_destroy(rx);
}

static void scenario_F(void)
{
    puts("F  D-STAR receive switches the chip's rate");
    voc_stream_t *rx = voc_stream_create(VOC_FMT_DSTAR, 1, "rx-dstar");
    talker_t t; talker_init(&t, VOC_FMT_DSTAR);
    uint8_t w[9]; int chip = 0;
    for (int k = 0; k < 60; k++) { talker_frame(&t, w); if (dec(rx, w).backend == VOC_BE_CHIP) chip++; sleep_ms(FRAME_MS); }
    pthread_mutex_lock(&sim_lock); int rate = sim_rate; pthread_mutex_unlock(&sim_lock);
    printf("   D-STAR on chip %d/60 frames, chip rate %s\n", chip, rate == 1 ? "D-STAR" : "DMR");
    CHECK(chip > 40 && rate == 1, "D-STAR did not get the chip at its rate");
    voc_stream_destroy(rx);
}

/* ---- G: stress ----------------------------------------------------------- */
struct worker { voc_fmt_t fmt; int decode; int seed; long frames, failures; };
static _Atomic int stress_stop;
static void *worker(void *p)
{
    struct worker *w = p;
    char id[16]; snprintf(id, sizeof id, "w%d", w->seed);
    voc_stream_t *s = voc_stream_create(w->fmt, w->decode, id);
    talker_t t; talker_init(&t, w->fmt);
    uint32_t r = (uint32_t)w->seed * 2654435761u;
    tls_stream = s;
    while (!stress_stop) {
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        int talking = ((r >> 8) % 100) < 60;
        int16_t pcm[160]; uint8_t wire[9]; voc_result_t res;
        int rc;
        if (w->decode) {
            if (talking) talker_frame(&t, wire); else for (int i = 0; i < 9; i++) wire[i] = (uint8_t)(r >> i);
            rc = voc_decode(s, ((r >> 3) & 31) == 0 ? NULL : wire, pcm, &res);
        } else {
            if (talking) speech(pcm, w->frames, 6000); else memset(pcm, 0, sizeof pcm);
            rc = voc_encode(s, pcm, wire, &res);
        }
        w->frames++; if (rc) w->failures++;
        if ((r & 7) == 0) sleep_ms(1);
    }
    voc_stream_destroy(s);
    return NULL;
}

static void scenario_G(void)
{
    puts("G  stress: 6 streams on 6 threads for 3 s, chip unplugged/replugged twice");
    struct worker ws[6] = {
        { VOC_FMT_DMR, 1, 1, 0, 0 }, { VOC_FMT_DMR, 1, 2, 0, 0 }, { VOC_FMT_YSF, 1, 3, 0, 0 },
        { VOC_FMT_DSTAR, 1, 4, 0, 0 }, { VOC_FMT_DMR, 0, 5, 0, 0 }, { VOC_FMT_DSTAR, 0, 6, 0, 0 } };
    pthread_t th[6];
    stress_stop = 0;
    pthread_mutex_lock(&sim_lock); int v0 = sim_violations, x0 = sim_xacts; pthread_mutex_unlock(&sim_lock);
    for (int i = 0; i < 6; i++) pthread_create(&th[i], NULL, worker, &ws[i]);
    sleep_ms(800); sim_set(0, 1); sleep_ms(400); sim_set(1, 0);
    sleep_ms(800); sim_set(0, 1); sleep_ms(300); sim_set(1, 0);
    sleep_ms(700);
    stress_stop = 1;
    long frames = 0, failures = 0;
    for (int i = 0; i < 6; i++) { pthread_join(th[i], NULL); frames += ws[i].frames; failures += ws[i].failures; }
    pthread_mutex_lock(&sim_lock);
    printf("   %ld frames, %ld failed, %d chip transactions, %d context violations\n",
           frames, failures, sim_xacts - x0, sim_violations - v0);
    CHECK(failures == 0, "%ld frames failed", failures);
    CHECK(sim_xacts - x0 > 100, "the chip was hardly used in the stress run");
    pthread_mutex_unlock(&sim_lock);
}

int main(void)
{
    sim_dec2450 = swdec_create(SWVOC_AMBE2450); sim_dec2400 = swdec_create(SWVOC_AMBE2400);
    sim_enc2450 = swenc_create(SWVOC_AMBE2450); sim_enc2400 = swenc_create(SWVOC_AMBE2400);
    static const struct voc_chip_ops ops = { sim_open_any, sim_close, sim_set_rate, sim_decode, sim_encode, sim_changed };
    voc_set_chip_ops(&ops);

    struct voc_config c;
    voc_config_defaults(&c);
    /* time scaled: one frame period = FRAME_MS ms */
    c.tx_hang_ms = 25 * FRAME_MS;     /* 25 frames */
    c.rx_hang_ms = 8 * FRAME_MS;      /* 8 frames  */
    c.verbose = getenv("VERBOSE") ? 1 : 0;
    CHECK(voc_init(&c) == 0, "init");
    wait_chip(1);

    scenario_A(); scenario_B(); scenario_C(); scenario_D(); scenario_E(); scenario_F(); scenario_G();

    pthread_mutex_lock(&sim_lock); int viol = sim_violations; pthread_mutex_unlock(&sim_lock);
    printf("chip context handed between streams without a reset: %d time(s)\n", viol);
    CHECK(viol == 0, "the chip's vocoder state was shared between streams");
    voc_shutdown();
    if (fails) { fprintf(stderr, "test_arbiter: %u failure(s)\n", fails); return 1; }
    puts("test_arbiter OK");
    return 0;
}
