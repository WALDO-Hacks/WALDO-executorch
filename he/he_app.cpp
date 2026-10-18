// WALDO two-brain, M55-HE image: the always-on ears. Same keyword spotter as the
// single-core build (src/kws.cpp: MFCC on the M55 as audio arrives, MicroNet-M every
// 200 ms), on the HE's Ethos-U55-128. Heard words, stats and a boot self-test go into the
// mailbox (src/waldo/brain_link.h) for the HP image. Between audio blocks the core sleeps.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>

#include <executorch/runtime/platform/runtime.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "he.h"
#include "kws.hpp"
#include "test_kws.h"
#include "test_wake.h"
#include "wake.hpp"
#include "waldo/brain_link.h"
#include "waldo/kws_mfcc.h"
#include "waldo/mic.h"

namespace {

brain_mailbox *const m = BRAIN_MAILBOX;

void set_stage(uint32_t stage, uint32_t error = 0)
{
    m->stage = stage;
    m->error = error;
    he_mailbox_flush();
}

// The recorded "yes" through the HE's MFCC and the U55: the HP prints the result.
void self_test()
{
    static float feats[KWS_FRAMES * KWS_COEFFS];
    const uint32_t t0 = DWT->CYCCNT;
    for (int f = 0; f < KWS_FRAMES; f++) kws_mfcc_frame(test_kws_yes + f * KWS_STRIDE, feats + f * KWS_COEFFS);
    const uint32_t mfcc_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
    float probs[KWS_LABELS];
    const int best = kws_classify(feats, probs);
    const bool pass = best >= 0 && std::string_view(kws_label(best)) == "yes";
    m->selftest_prob = best >= 0 ? probs[best] : 0.0f;
    m->selftest_mfcc_us = mfcc_us;
    m->selftest_npu_us = kws_last_us();
    m->selftest = pass ? 1 : 0;
    printf("HE: self-test heard \"%s\" (%.2f), MFCC %lu us, NPU %lu us\n", kws_label(best),
           (double)m->selftest_prob, (unsigned long)mfcc_us, (unsigned long)kws_last_us());
}

// The wake word: a known "Hey Waldo" must score high and the KWS "yes" low, on the U55.
// Failure only disables the wake word (the HP keeps using the command words).
void wake_boot(bool loaded)
{
    float p_pos = -1.0f, p_neg = -1.0f;
    const bool pass = loaded && wake_self_test(test_wake_pos, test_kws_yes, &p_pos, &p_neg);
    printf("HE: wake word %s: \"Hey Waldo\" %.3f (host float %.3f), \"yes\" %.3f, NPU %lu us\n",
           pass ? "enabled" : "DISABLED", (double)p_pos, (double)test_wake_float_prob, (double)p_neg,
           (unsigned long)wake_last_us());
}

}  // namespace

extern "C" int he_app_main(void)
{
    printf("HE: M55-HE at %lu MHz, two-brain ears starting\n", (unsigned long)(SystemCoreClock / 1000000u));
    if (ethos_setup() != 0) {
        set_stage(BRAIN_NPU, 1);
        for (;;) __WFI();
    }
    set_stage(BRAIN_NPU);

    executorch::runtime::runtime_init();
    if (!kws_init()) {
        printf("HE: keyword spotter init failed\n");
        set_stage(BRAIN_MODEL, 1);
        for (;;) __WFI();
    }
    set_stage(BRAIN_MODEL);
    const bool wake_loaded = wake_init();

    self_test();
    wake_boot(wake_loaded);
    set_stage(BRAIN_SELFTEST);

    if (mic_init() != 0) {
        set_stage(BRAIN_MIC, 1);
        for (;;) __WFI();
    }
    set_stage(BRAIN_LISTENING);

    uint32_t last_stats = he_millis();
    for (;;) {
        const uint32_t now = he_millis();
        const int word = kws_listen(now);
        if (word >= 0) {
            brain_post_word(m, word);
            he_mailbox_flush();
            printf("HE: heard %s\n", kws_label(word));
        }
        if (now - last_stats >= 1000) {
            uint32_t mfcc_us, npu_us, windows;
            kws_listen_stats(&mfcc_us, &npu_us, &windows);
            float r0, r1;
            uint32_t errs;
            mic_levels(&r0, &r1, &errs);
            m->windows += windows;
            m->mfcc_us += mfcc_us;
            uint32_t wake_us, wake_windows, wake_events;
            wake_stats(&wake_us, &wake_windows, &wake_events);
            m->npu_us += npu_us + wake_us;   // both networks' U55 time
            m->rms0 = r0;
            m->rms1 = r1;
            m->mic_errors = errs;
            m->heartbeat++;
            he_mailbox_flush();
            last_stats = now;
        }
        // Nothing to do until the next audio block (every 16 ms) or tick: sleep.
        __WFI();
    }
}
