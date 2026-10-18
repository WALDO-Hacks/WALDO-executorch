// MicroNet-M keyword spotting on the Ethos-U85 (kws.hpp). The NPU runs the whole network
// (model/kws/micronet.py, 42 operators); the Cortex-M55 computes the MFCCs as audio
// arrives (src/waldo/kws_mfcc.c) and smooths the decisions.

#include "kws.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

#include <executorch/extension/data_loader/buffer_data_loader.h>
#include <executorch/runtime/core/evalue.h>
#include <executorch/runtime/core/exec_aten/exec_aten.h>
#include <executorch/runtime/core/memory_allocator.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "arm_embedded_module.hpp"
#include "kws_pte.h"
#include "waldo/kws_mfcc.h"
#include "waldo/mic.h"

using arm::embedded::EmbeddedModule;
using executorch::aten::DimOrderType;
using executorch::aten::ScalarType;
using executorch::aten::SizesType;
using executorch::aten::Tensor;
using executorch::aten::TensorImpl;
using executorch::extension::BufferDataLoader;
using executorch::runtime::MemoryAllocator;

namespace {

// The two-brain HE image (he/) sets its own section and sizes: the U55-128 needs 121 KiB.
#if defined(WALDO_HE_POOLS_DTCM)
// he/ fallback: both pools in the HE's DTCM (plain .bss), as in Alif's own HE examples;
// the U55 reaches it at the global alias (he/ethos_setup.c). Sized to kws_pte_he.c:
// planned memory 2464 B + method metadata ~1 KB (3472 B measured on a 64-bit host),
// Vela scratch 123600 B. The absolute symbol tells he/linker_gnu_he.ld.src to shrink the
// heaps to make room.
#define KWS_POOL_ATTRIBUTES
#undef KWS_METHOD_POOL_SIZE
#undef KWS_TEMP_POOL_SIZE
#define KWS_METHOD_POOL_SIZE (8 * 1024)
#define KWS_TEMP_POOL_SIZE (124 * 1024)
#if !defined(MAX_KERNEL_NUM)
#error "WALDO_HE_POOLS_DTCM needs MAX_KERNEL_NUM too (he/waldo-he.cproject.yml): DTCM is full otherwise"
#endif
__asm__(".global __waldo_he_pools_dtcm\n\t.set __waldo_he_pools_dtcm, 1");
#elif defined(KWS_POOL_SECTION)
#define KWS_POOL_ATTRIBUTES __attribute__((section(KWS_POOL_SECTION)))
#elif defined(APP_POOL_SECTION)
#define KWS_POOL_ATTRIBUTES __attribute__((section(APP_POOL_SECTION)))
#else
#define KWS_POOL_ATTRIBUTES
#endif
#ifndef KWS_METHOD_POOL_SIZE
#define KWS_METHOD_POOL_SIZE (128 * 1024)
#endif
#ifndef KWS_TEMP_POOL_SIZE
#define KWS_TEMP_POOL_SIZE (128 * 1024)   // Ethos-U scratch (Vela: 55 KB on the U85)
#endif
constexpr size_t kMethodPool = KWS_METHOD_POOL_SIZE;
constexpr size_t kTempPool = KWS_TEMP_POOL_SIZE;
alignas(16) uint8_t g_method_pool[kMethodPool] KWS_POOL_ATTRIBUTES;
alignas(16) uint8_t g_temp_pool[kTempPool] KWS_POOL_ATTRIBUTES;

const char *const kLabels[KWS_LABELS] = KWS_LABEL_NAMES;
constexpr int kWords = KWS_WORDS;        // labels before _silence_ / _unknown_
// Per word: off (a word that failed its test stays in the model but is never reported) and
// the averaged probability it needs, in hundredths (src/kws_labels.h).
constexpr unsigned char kEnabled[KWS_LABELS] = KWS_LABEL_ENABLED;
constexpr unsigned char kThresholdPct[KWS_LABELS] = KWS_LABEL_THRESHOLD_PCT;

std::unique_ptr<EmbeddedModule> g_module;
alignas(16) float g_input[KWS_FRAMES * KWS_COEFFS];
uint32_t g_last_us;
#if defined(WALDO_HE)
MemoryAllocator *g_method_alloc;   // for the one-off usage line after the first forward
#endif

inline uint32_t cyc_to_us(uint32_t c) { return c / (SystemCoreClock / 1000000u); }

}  // namespace

bool kws_init()
{
    // Cycle counter for the stage timing.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    auto method_alloc = std::make_unique<MemoryAllocator>(kMethodPool, g_method_pool);
#if defined(WALDO_HE)
    g_method_alloc = method_alloc.get();
#endif
    g_module = std::make_unique<EmbeddedModule>(
        kws_pte, kws_pte_size, std::make_unique<BufferDataLoader>(kws_pte, kws_pte_size),
        std::move(method_alloc), std::make_unique<MemoryAllocator>(kTempPool, g_temp_pool));
#if defined(WALDO_HE)
    printf("HE: KWS pools at %p (method %u B) and %p (temp %u B)\n", (void *)g_method_pool,
           (unsigned)kMethodPool, (void *)g_temp_pool, (unsigned)kTempPool);
#endif
    kws_mfcc_init();
    return g_module != nullptr;
}

const char *kws_label(int i) { return i >= 0 && i < KWS_LABELS ? kLabels[i] : "?"; }
uint32_t kws_last_us() { return g_last_us; }

int kws_classify(const float *mfcc, float *probs)
{
    if (!g_module) return -1;
    std::memcpy(g_input, mfcc, sizeof(g_input));
    std::array<SizesType, 4> sizes{1, KWS_FRAMES, KWS_COEFFS, 1};
    std::array<DimOrderType, 4> dim_order{0, 1, 2, 3};
    TensorImpl impl(ScalarType::Float, sizes.size(), sizes.data(), g_input, dim_order.data());
    Tensor input(&impl);

    const uint32_t t0 = DWT->CYCCNT;
    auto outputs = g_module->forward(input);
    g_last_us = cyc_to_us(DWT->CYCCNT - t0);
#if defined(WALDO_HE)
    if (g_method_alloc) {
        printf("HE: method pool %u of %u B used\n", (unsigned)g_method_alloc->used_size(), (unsigned)kMethodPool);
        g_method_alloc = nullptr;
    }
#endif
    if (!outputs.ok() || outputs->empty() || !outputs->front().isTensor()) {
        printf("kws: forward failed (err=%u)\n", outputs.ok() ? 0u : (unsigned)outputs.error());
        return -1;
    }
    const Tensor t = outputs->front().toTensor();
    if (t.scalar_type() != ScalarType::Float || t.numel() != KWS_LABELS) {
        printf("kws: unexpected output (%u elements)\n", (unsigned)t.numel());
        return -1;
    }
    const float *logits = t.const_data_ptr<float>();
    float mx = logits[0], sum = 0;
    for (int i = 1; i < KWS_LABELS; i++) mx = std::fmax(mx, logits[i]);
    int best = 0;
    for (int i = 0; i < KWS_LABELS; i++) {
        probs[i] = std::exp(logits[i] - mx);
        sum += probs[i];
    }
    for (int i = 0; i < KWS_LABELS; i++) {
        probs[i] /= sum;
        if (probs[i] > probs[best]) best = i;
    }
    return best;
}

// ---------- live listening ----------

namespace {
constexpr int kClassifyEvery = 10;      // MFCC frames (20 ms each) between classifications
constexpr int kSmooth = 3;              // classifications averaged
// Averaged probability to accept a word: kThresholdPct (0.80 for MicroNet: half the false
// triggers of 0.70 in the TTS test, same hit rate).
constexpr uint32_t kRepeatMs = 1000;    // the same word again only after this

int16_t g_frame[KWS_FRAME];             // the last 640 samples
float g_feats[KWS_FRAMES][KWS_COEFFS];  // rolling window, oldest first
int g_frames_seen, g_since_classify;
float g_hist[kSmooth][KWS_LABELS];
int g_hist_n;
int g_last_word = -1;
uint32_t g_last_word_ms;
uint32_t g_mfcc_us, g_npu_us, g_windows;
}  // namespace

int kws_listen(uint32_t now_ms)
{
    static int16_t pending[KWS_STRIDE];
    static int pending_n, hops;
    int heard = -1;
    for (;;) {
        // Every 320 new samples slide the 640-sample frame along and add one MFCC frame.
        pending_n += (int)mic_read(pending + pending_n, KWS_STRIDE - pending_n);
        if (pending_n < KWS_STRIDE) break;
        std::memmove(g_frame, g_frame + KWS_STRIDE, (KWS_FRAME - KWS_STRIDE) * sizeof(int16_t));
        std::memcpy(g_frame + KWS_FRAME - KWS_STRIDE, pending, sizeof(pending));
        pending_n = 0;
        if (hops < 2) {   // the first frame needs two hops of audio
            hops++;
            if (hops < 2) continue;
        }

        uint32_t t0 = DWT->CYCCNT;
        std::memmove(g_feats[0], g_feats[1], sizeof(g_feats) - sizeof(g_feats[0]));
        kws_mfcc_frame(g_frame, g_feats[KWS_FRAMES - 1]);
        g_mfcc_us += cyc_to_us(DWT->CYCCNT - t0);
        if (g_frames_seen < KWS_FRAMES) g_frames_seen++;
        if (g_frames_seen < KWS_FRAMES || ++g_since_classify < kClassifyEvery) continue;
        g_since_classify = 0;

        float probs[KWS_LABELS];
        if (kws_classify(&g_feats[0][0], probs) < 0) continue;
        g_npu_us += g_last_us;
        g_windows++;
        std::memmove(g_hist[1], g_hist[0], sizeof(g_hist) - sizeof(g_hist[0]));
        std::memcpy(g_hist[0], probs, sizeof(probs));
        if (g_hist_n < kSmooth) g_hist_n++;

        int best = 0;
        float avg[KWS_LABELS];
        for (int i = 0; i < KWS_LABELS; i++) {
            avg[i] = 0;
            for (int h = 0; h < g_hist_n; h++) avg[i] += g_hist[h][i];
            avg[i] /= g_hist_n;
            if (avg[i] > avg[best]) best = i;
        }
        if (best < kWords && kEnabled[best] && avg[best] * 100.0f >= kThresholdPct[best] &&
            (best != g_last_word || now_ms - g_last_word_ms > kRepeatMs)) {
            g_last_word = best;
            g_last_word_ms = now_ms;
            heard = best;
            g_hist_n = 0;   // a fresh start for the next word
        } else if (best < kWords && probs[best] > 0.5f) {
            printf("kws: maybe %s (%.2f, avg %.2f)\n", kLabels[best], (double)probs[best], (double)avg[best]);
        }
    }
    return heard;
}

void kws_listen_stats(uint32_t *mfcc_us, uint32_t *npu_us, uint32_t *windows)
{
    *mfcc_us = g_mfcc_us;
    *npu_us = g_npu_us;
    *windows = g_windows;
    g_mfcc_us = g_npu_us = g_windows = 0;
}
