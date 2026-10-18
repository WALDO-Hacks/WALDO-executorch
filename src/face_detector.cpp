// BlazeFace on the Ethos-U85 through ExecuTorch. The NPU runs the whole network
// (model/model.py: 128x128x1 in, (1, 896, 5) out); the Cortex-M55 letterboxes the
// input and decodes the 896 anchors into boxes with non-max suppression.

#include "face_detector.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <memory>

#include <executorch/extension/data_loader/buffer_data_loader.h>
#include <executorch/runtime/core/evalue.h>
#include <executorch/runtime/core/exec_aten/exec_aten.h>
#include <executorch/runtime/core/memory_allocator.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "arm_embedded_module.hpp"
#include "blazeface_anchors.h"
#include "model_pte.h"

using arm::embedded::EmbeddedModule;
using executorch::aten::DimOrderType;
using executorch::aten::ScalarType;
using executorch::aten::SizesType;
using executorch::aten::Tensor;
using executorch::aten::TensorImpl;
using executorch::extension::BufferDataLoader;
using executorch::runtime::EValue;
using executorch::runtime::MemoryAllocator;

namespace {

#ifndef APP_METHOD_POOL_SIZE
#define APP_METHOD_POOL_SIZE (4 * 1024 * 1024)
#endif
#ifndef APP_TEMP_POOL_SIZE
#define APP_TEMP_POOL_SIZE (4 * 1024 * 1024)  // Ethos-U scratch is drawn from here.
#endif
#ifdef APP_POOL_SECTION
#define APP_POOL_ATTRIBUTES __attribute__((section(APP_POOL_SECTION)))
#else
#define APP_POOL_ATTRIBUTES
#endif

// The NPU reaches SRAM, not the core's DTCM: the pools go where APP_POOL_SECTION says.
alignas(16) uint8_t g_method_pool[APP_METHOD_POOL_SIZE] APP_POOL_ATTRIBUTES;
alignas(16) uint8_t g_temp_pool[APP_TEMP_POOL_SIZE] APP_POOL_ATTRIBUTES;

constexpr int kSize = 128;            // model input, square
constexpr int kAnchors = 896;
constexpr float kScoreThresh = 0.75f; // MediaPipe's front-model threshold
constexpr float kNmsIou = 0.3f;

std::unique_ptr<EmbeddedModule> g_module;
alignas(16) float g_input[kSize * kSize];
uint32_t g_last_us, g_pre_us, g_post_us;

uint32_t cyc_to_us(uint32_t c) { return c / (SystemCoreClock / 1000000u); }

float iou(const face_box &a, const face_box &b)
{
    float ix = std::fmax(0.f, std::fmin(a.x + a.w, b.x + b.w) - std::fmax(a.x, b.x));
    float iy = std::fmax(0.f, std::fmin(a.y + a.h, b.y + b.h) - std::fmax(a.y, b.y));
    float inter = ix * iy;
    return inter / (a.w * a.h + b.w * b.h - inter + 1e-6f);
}

}  // namespace

bool face_detector_init()
{
    g_module = std::make_unique<EmbeddedModule>(
        model_pte, model_pte_size, std::make_unique<BufferDataLoader>(model_pte, model_pte_size),
        std::make_unique<MemoryAllocator>(APP_METHOD_POOL_SIZE, g_method_pool),
        std::make_unique<MemoryAllocator>(APP_TEMP_POOL_SIZE, g_temp_pool));
    // Cycle counter for timing the NPU call.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return g_module != nullptr;
}

uint32_t face_detector_last_us() { return g_last_us; }

void face_detector_stage_us(uint32_t *pre, uint32_t *npu, uint32_t *post)
{
    *pre = g_pre_us;
    *npu = g_last_us;
    *post = g_post_us;
}

int face_detect_gray(const uint8_t *img, int w, int h, face_box *out, int max)
{
    if (!g_module || w <= 0 || h <= 0 || w > 1024 || h > 1024) return -1;

    const uint32_t t_pre = DWT->CYCCNT;
    // Letterbox: scale the longer side to 128, centre, pad with mid-grey (0.0).
    const float scale = (float)kSize / (w > h ? w : h);
    const int sw = (int)(w * scale + 0.5f), sh = (int)(h * scale + 0.5f);
    const int ox = (kSize - sw) / 2, oy = (kSize - sh) / 2;
    for (int y = 0; y < kSize; y++) {
        for (int x = 0; x < kSize; x++) {
            int u = x - ox, v = y - oy;
            float p = 0.0f;
            if (u >= 0 && u < sw && v >= 0 && v < sh) {
                int sx = (int)(u / scale), sy = (int)(v / scale);
                if (sx >= w) sx = w - 1;
                if (sy >= h) sy = h - 1;
                p = img[sy * w + sx] / 127.5f - 1.0f;
            }
            g_input[y * kSize + x] = p;
        }
    }

    std::array<SizesType, 4> sizes{1, kSize, kSize, 1};
    std::array<DimOrderType, 4> dim_order{0, 1, 2, 3};
    TensorImpl impl(ScalarType::Float, sizes.size(), sizes.data(), g_input, dim_order.data());
    Tensor input(&impl);

    uint32_t t0 = DWT->CYCCNT;
    g_pre_us = cyc_to_us(t0 - t_pre);
    auto outputs = g_module->forward(input);
    const uint32_t t_post = DWT->CYCCNT;
    g_last_us = cyc_to_us(t_post - t0);
    if (!outputs.ok() || outputs->empty() || !outputs->front().isTensor()) {
        printf("face: forward failed (err=%u)\n", outputs.ok() ? 0u : (unsigned)outputs.error());
        return -1;
    }
    const Tensor t = outputs->front().toTensor();
    if (t.scalar_type() != ScalarType::Float || t.numel() != kAnchors * 5) {
        printf("face: unexpected output (%u elements)\n", (unsigned)t.numel());
        return -1;
    }
    const float *o = t.const_data_ptr<float>();

    // Decode: per anchor [score logit, dx, dy, dw, dh]; offsets are in 128-px units
    // relative to the anchor centre. Keep the best kScoreThresh-passing boxes, then NMS.
    constexpr int kCand = 32;
    face_box cand[kCand];
    int nc = 0;
    for (int i = 0; i < kAnchors; i++) {
        const float *a = o + i * 5;
        float logit = a[0] < -100.f ? -100.f : (a[0] > 100.f ? 100.f : a[0]);
        float s = 1.0f / (1.0f + std::exp(-logit));
        if (s < kScoreThresh) continue;
        float cx = a[1] / kSize + blazeface_anchors[i][0];
        float cy = a[2] / kSize + blazeface_anchors[i][1];
        float bw = a[3] / kSize, bh = a[4] / kSize;
        face_box b{cx - bw / 2, cy - bh / 2, bw, bh, s};   // normalised to the 128 input
        if (nc < kCand) {
            cand[nc++] = b;
        } else {
            int worst = 0;
            for (int k = 1; k < kCand; k++) if (cand[k].score < cand[worst].score) worst = k;
            if (b.score > cand[worst].score) cand[worst] = b;
        }
    }
    // Sort by score (tiny n), then greedy NMS.
    for (int i = 1; i < nc; i++)
        for (int k = i; k > 0 && cand[k].score > cand[k - 1].score; k--) std::swap(cand[k], cand[k - 1]);
    int n = 0;
    for (int i = 0; i < nc && n < max; i++) {
        bool keep = true;
        for (int k = 0; k < n && keep; k++) {
            face_box kb{(out[k].x * scale + ox) / kSize, (out[k].y * scale + oy) / kSize,
                        out[k].w * scale / kSize, out[k].h * scale / kSize, 0};
            keep = iou(cand[i], kb) < kNmsIou;
        }
        if (!keep) continue;
        // Back to the caller's image pixels.
        out[n++] = {(cand[i].x * kSize - ox) / scale, (cand[i].y * kSize - oy) / scale,
                    cand[i].w * kSize / scale, cand[i].h * kSize / scale, cand[i].score};
    }
    g_post_us = cyc_to_us(DWT->CYCCNT - t_post);
    return n;
}
