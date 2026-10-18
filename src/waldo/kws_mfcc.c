// MFCC front end of the keyword spotter (see kws_mfcc.h). Plain C on the M55's FPU:
// about 0.15 ms per 20 ms of audio, so it keeps up with the microphones as they record.

#include "kws_mfcc.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define FFT_N   1024
#define FFT_LOG 10
#define BINS    (FFT_N / 2 + 1)
#define BANDS   40
#define PI_F    3.14159265358979f

static float hann[KWS_FRAME];
static float tw_cos[FFT_N / 2], tw_sin[FFT_N / 2];
static uint16_t bitrev[FFT_N];
static float re[FFT_N], im[FFT_N], mag[BINS];

// Mel bands as runs of weights: band b covers bins first[b]..first[b]+len[b]-1.
static uint16_t band_first[BANDS], band_len[BANDS], band_off[BANDS];
static float band_w[BINS * 2];
static float dct[KWS_COEFFS][BANDS];

static float mel(float f) { return 1127.0f * logf(1.0f + f / 700.0f); }

void kws_mfcc_init(void)
{
    for (int i = 0; i < KWS_FRAME; i++) hann[i] = 0.5f - 0.5f * cosf(i * 2 * PI_F / KWS_FRAME);
    for (int i = 0; i < FFT_N / 2; i++) {
        tw_cos[i] = cosf(2 * PI_F * i / FFT_N);
        tw_sin[i] = -sinf(2 * PI_F * i / FFT_N);
    }
    for (int i = 0; i < FFT_N; i++) {
        unsigned r = 0;
        for (int b = 0; b < FFT_LOG; b++) r |= ((i >> b) & 1u) << (FFT_LOG - 1 - b);
        bitrev[i] = (uint16_t)r;
    }

    const float lo = mel(20.0f), hi = mel(4000.0f), delta = (hi - lo) / (BANDS + 1);
    int off = 0;
    for (int b = 0; b < BANDS; b++) {
        const float l = lo + b * delta, c = lo + (b + 1) * delta, r = lo + (b + 2) * delta;
        band_first[b] = 0;
        band_len[b] = 0;
        band_off[b] = (uint16_t)off;
        for (int i = 0; i < FFT_N / 2; i++) {
            const float m = mel(16000.0f * i / FFT_N);
            if (m > l && m < r) {
                if (band_len[b] == 0) band_first[b] = (uint16_t)i;
                band_w[off++] = m <= c ? (m - l) / (c - l) : (r - m) / (r - c);
                band_len[b]++;
            }
        }
    }
    const float norm = sqrtf(2.0f / BANDS);
    for (int k = 0; k < KWS_COEFFS; k++)
        for (int n = 0; n < BANDS; n++) dct[k][n] = norm * cosf((n + 0.5f) * k * PI_F / BANDS);
}

// In-place iterative radix-2 FFT of re/im (already in bit-reversed order).
static void fft(void)
{
    for (int size = 2; size <= FFT_N; size <<= 1) {
        const int half = size >> 1, step = FFT_N / size;
        for (int start = 0; start < FFT_N; start += size) {
            for (int k = 0; k < half; k++) {
                const float wr = tw_cos[k * step], wi = tw_sin[k * step];
                const int a = start + k, b = a + half;
                const float tr = re[b] * wr - im[b] * wi, ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
    }
}

void kws_mfcc_frame(const int16_t *frame, float *out)
{
    for (int i = 0; i < FFT_N; i++) {
        const int j = bitrev[i];
        re[i] = j < KWS_FRAME ? frame[j] * (1.0f / 32768.0f) * hann[j] : 0.0f;
        im[i] = 0.0f;
    }
    fft();
    for (int i = 0; i < BINS; i++) mag[i] = sqrtf(re[i] * re[i] + im[i] * im[i]);

    float e[BANDS];
    for (int b = 0; b < BANDS; b++) {
        float s = FLT_MIN;
        const float *w = &band_w[band_off[b]];
        for (int i = 0; i < band_len[b]; i++) s += w[i] * mag[band_first[b] + i];
        e[b] = logf(s);
    }
    for (int k = 0; k < KWS_COEFFS; k++) {
        float s = 0;
        for (int n = 0; n < BANDS; n++) s += dct[k][n] * e[n];
        out[k] = s;
    }
}
