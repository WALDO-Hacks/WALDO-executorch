// MFCC features for the MicroNet keyword spotter, the same maths as model/kws/mfcc.py and
// Alif's ML Embedded Evaluation Kit: a 640-sample (40 ms) frame at 16 kHz -> Hann window,
// 1024-point FFT, magnitude, 40 HTK mel bands (20-4000 Hz), log, DCT -> 10 coefficients.
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define KWS_FRAME   640
#define KWS_STRIDE  320
#define KWS_COEFFS  10
#define KWS_FRAMES  49
#define KWS_WINDOW  (KWS_FRAMES * KWS_STRIDE + KWS_FRAME - KWS_STRIDE)   // 16000 = 1 s

void kws_mfcc_init(void);
void kws_mfcc_frame(const int16_t *frame, float *out);   // KWS_FRAME samples -> KWS_COEFFS

#ifdef __cplusplus
}
#endif
