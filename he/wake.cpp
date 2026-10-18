// "Hey Waldo" wake word on the M55-HE (wake.hpp): a second ExecuTorch program on the same
// Ethos-U55-128 as MicroNet-M, in its own pools in the HE's part of SRAM0.

#include "wake.hpp"

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

#include "../src/arm_embedded_module.hpp"
#include "he.h"
#include "wake_config.h"
#include "wake_debounce.h"
#include "waldo/brain_link.h"
#include "waldo/kws_mfcc.h"

extern "C" {
extern const unsigned char wake_pte[];
extern const unsigned long wake_pte_size;
}

using arm::embedded::EmbeddedModule;
using executorch::aten::DimOrderType;
using executorch::aten::ScalarType;
using executorch::aten::SizesType;
using executorch::aten::Tensor;
using executorch::aten::TensorImpl;
using executorch::extension::BufferDataLoader;
using executorch::runtime::MemoryAllocator;

namespace {

// Sized to the Vela report of wake_pte_he.c (WAKE_*_POOL_SIZE in he/wake_config.h).
alignas(16) uint8_t g_method_pool[WAKE_METHOD_POOL_SIZE] __attribute__((section(".bss.he_pool")));
alignas(16) uint8_t g_temp_pool[WAKE_TEMP_POOL_SIZE] __attribute__((section(".bss.he_pool")));

std::unique_ptr<EmbeddedModule> g_module;
alignas(16) float g_input[KWS_FRAMES * KWS_COEFFS];
uint32_t g_last_us, g_npu_us, g_windows, g_events;
bool g_enabled;
wake_debounce g_deb;
MemoryAllocator *g_method_alloc;   // for the one-off usage line after the first forward

inline uint32_t cyc_to_us(uint32_t c) { return c / (SystemCoreClock / 1000000u); }

void mfcc_window(const int16_t *audio, float *feats)
{
    for (int f = 0; f < KWS_FRAMES; f++) kws_mfcc_frame(audio + f * KWS_STRIDE, feats + f * KWS_COEFFS);
}

}  // namespace

bool wake_init()
{
    auto method_alloc = std::make_unique<MemoryAllocator>(sizeof g_method_pool, g_method_pool);
    g_method_alloc = method_alloc.get();
    g_module = std::make_unique<EmbeddedModule>(
        wake_pte, wake_pte_size, std::make_unique<BufferDataLoader>(wake_pte, wake_pte_size),
        std::move(method_alloc), std::make_unique<MemoryAllocator>(sizeof g_temp_pool, g_temp_pool));
    printf("HE: wake pools at %p (method %u B) and %p (temp %u B), program %lu B\n", (void *)g_method_pool,
           (unsigned)sizeof g_method_pool, (void *)g_temp_pool, (unsigned)sizeof g_temp_pool, wake_pte_size);
    return g_module != nullptr;
}

float wake_classify(const float *mfcc)
{
    if (!g_module) return -1.0f;
    std::memcpy(g_input, mfcc, sizeof(g_input));
    std::array<SizesType, 4> sizes{1, KWS_FRAMES, KWS_COEFFS, 1};
    std::array<DimOrderType, 4> dim_order{0, 1, 2, 3};
    TensorImpl impl(ScalarType::Float, sizes.size(), sizes.data(), g_input, dim_order.data());
    Tensor input(&impl);

    const uint32_t t0 = DWT->CYCCNT;
    auto outputs = g_module->forward(input);
    g_last_us = cyc_to_us(DWT->CYCCNT - t0);
    if (g_method_alloc) {
        printf("HE: wake method pool %u of %u B used\n", (unsigned)g_method_alloc->used_size(), (unsigned)sizeof g_method_pool);
        g_method_alloc = nullptr;
    }
    if (!outputs.ok() || outputs->empty() || !outputs->front().isTensor()) {
        printf("wake: forward failed (err=%u)\n", outputs.ok() ? 0u : (unsigned)outputs.error());
        return -1.0f;
    }
    const Tensor t = outputs->front().toTensor();
    if (t.scalar_type() != ScalarType::Float || t.numel() != 2) {
        printf("wake: unexpected output (%u elements)\n", (unsigned)t.numel());
        return -1.0f;
    }
    const float *logits = t.const_data_ptr<float>();
    return 1.0f / (1.0f + std::exp(logits[0] - logits[1]));   // softmax over [other, wake]
}

bool wake_self_test(const int16_t *positive, const int16_t *negative, float *p_pos, float *p_neg)
{
    static float feats[KWS_FRAMES * KWS_COEFFS];
    mfcc_window(positive, feats);
    *p_pos = wake_classify(feats);
    mfcc_window(negative, feats);
    *p_neg = wake_classify(feats);
    const bool pass = *p_pos >= 0.5f && *p_neg >= 0.0f && *p_neg < 0.5f;
    if (pass) {
        BRAIN_MAILBOX->wake_enabled = 1;
        he_mailbox_flush();
    }
    return pass;
}

void wake_on_window(const float *mfcc, uint32_t now_ms)
{
    if (!g_enabled && BRAIN_MAILBOX->wake_enabled != 1) return;
    g_enabled = true;
    const float p = wake_classify(mfcc);
    if (p < 0.0f) return;
    g_npu_us += g_last_us;
    g_windows++;
    if (wake_debounce_step(&g_deb, p >= WAKE_THRESHOLD, WAKE_N_WINDOW, WAKE_K_HITS, WAKE_REFRACTORY_WINDOWS)) {
        g_events++;
        brain_post_wake(BRAIN_MAILBOX, (uint32_t)(p * 1000.0f + 0.5f));
        he_mailbox_flush();
        printf("HE: wake word (%.3f) at %lu ms\n", (double)p, (unsigned long)now_ms);
    }
}

uint32_t wake_last_us() { return g_last_us; }

void wake_stats(uint32_t *npu_us, uint32_t *windows, uint32_t *events)
{
    *npu_us = g_npu_us;
    *windows = g_windows;
    *events = g_events;
    g_npu_us = g_windows = g_events = 0;
}
