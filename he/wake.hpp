// "Hey Waldo" wake word on the HE's Ethos-U55 beside MicroNet-M (model/wake/, documentation/wake-word.md).
// Same 49 x 10 MFCC window as the keyword spotter; a second ExecuTorch program (wake_pte) with
// its own pools. Runs once per 200 ms window, debounces, and posts events to the mailbox.
#pragma once
#include <stdint.h>

// Loads wake_pte. Call once, after runtime_init(). False: the model failed to load.
bool wake_init();
// Probability of "Hey Waldo" for one window of 49 x 10 MFCCs; negative on error.
float wake_classify(const float *mfcc);
// Boot self-test on two 1 s int16 windows (a known positive, a known negative such as the
// KWS "yes"): on pass the mailbox's wake_enabled becomes 1. Returns pass.
bool wake_self_test(const int16_t *positive, const int16_t *negative, float *p_pos, float *p_neg);
// Called with every new window's MFCCs (from kws_listen); posts a wake event when the
// debounce fires. Does nothing until the self-test passed.
void wake_on_window(const float *mfcc, uint32_t now_ms);
// Microseconds the last NPU inference took, and the totals since the last call.
uint32_t wake_last_us();
void wake_stats(uint32_t *npu_us, uint32_t *windows, uint32_t *events);
