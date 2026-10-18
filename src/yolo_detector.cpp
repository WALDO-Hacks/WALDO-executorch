// YOLOv8n on the Ethos-U85 (yolo_detector.hpp). The NPU runs the whole network (99
// operators, model/yolo/yolo.py); the Cortex-M55 letterboxes the frame into the float
// input and decodes the three raw head maps (src/yolo/waldo_yolo_decode.h).

#include "yolo_detector.hpp"

#include <array>
#include <cstdio>
#include <memory>

#include <executorch/extension/data_loader/buffer_data_loader.h>
#include <executorch/runtime/core/evalue.h>
#include <executorch/runtime/core/exec_aten/exec_aten.h>
#include <executorch/runtime/core/memory_allocator.h>

#include "RTE_Components.h"
#include CMSIS_device_header

extern "C" uint32_t ethos_npu_cycles_take(void);   // board/DevKit-E8/ethos_setup.c

#include "arm_embedded_module.hpp"
#include "yolo/waldo_yolo_classes.h"
#include "yolo/waldo_yolo_decode.h"

extern "C" const unsigned char yolo_pte[];
extern "C" const unsigned long yolo_pte_size;

using arm::embedded::EmbeddedModule;
using executorch::aten::DimOrderType;
using executorch::aten::ScalarType;
using executorch::aten::SizesType;
using executorch::aten::Tensor;
using executorch::aten::TensorImpl;
using executorch::extension::BufferDataLoader;
using executorch::runtime::MemoryAllocator;

namespace {

constexpr int kSize = 320;
constexpr float kNmsIou = 0.7f;   // the team's evaluation setting
static_assert(waldo::kNumClasses == YOLO_CLASSES, "class count");

// Measured for this export: 1,536,000 B planned (input + head maps), 2005 KiB Vela scratch.
// Fallback if the NPU turns out not to reach SRAM1: build with WALDO_NPU_SRAM0_ONLY and
// both pools sit in SRAM0 (the LCD frame buffer moves to SRAM1 to make room).
constexpr size_t kMethodPool = 0x1A0000;   // 1.6 MB, SRAM0
constexpr size_t kTempPool = 0x204000;     // 2.0 MB, SRAM1 (SRAM0 with WALDO_NPU_SRAM0_ONLY)
#ifdef WALDO_NPU_SRAM0_ONLY
#define YOLO_TEMP_SECTION ".bss.ai_pool"
#else
#define YOLO_TEMP_SECTION ".bss.sram1.yolo_temp"
#endif
alignas(16) uint8_t g_method_pool[kMethodPool] __attribute__((section(".bss.ai_pool")));
alignas(16) uint8_t g_temp_pool[kTempPool] __attribute__((section(YOLO_TEMP_SECTION)));
alignas(16) float g_input[3 * kSize * kSize] __attribute__((section(".bss.sram1.yolo_input")));

std::unique_ptr<EmbeddedModule> g_module;
waldo::Det g_scratch[waldo::kMaxCandidates];
uint32_t g_pre_us, g_npu_us, g_post_us, g_hw_us;

inline uint32_t cyc_to_us(uint32_t c) { return c / (SystemCoreClock / 1000000u); }

}  // namespace

bool yolo_init()
{
    // Cycle counter for the stage timing.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_module = std::make_unique<EmbeddedModule>(
        yolo_pte, yolo_pte_size, std::make_unique<BufferDataLoader>(yolo_pte, yolo_pte_size),
        std::make_unique<MemoryAllocator>(kMethodPool, g_method_pool),
        std::make_unique<MemoryAllocator>(kTempPool, g_temp_pool));
    return g_module != nullptr;
}

const char *yolo_class_name(int cls)
{
    return cls >= 0 && cls < YOLO_CLASSES ? waldo::kClassNames[cls] : "?";
}

uint32_t yolo_npu_hw_us() { return g_hw_us; }

void yolo_stage_us(uint32_t *pre, uint32_t *npu, uint32_t *post)
{
    *pre = g_pre_us;
    *npu = g_npu_us;
    *post = g_post_us;
}

int yolo_detect_rgb(const uint8_t *rgb, int w, int h, float min_score, yolo_box *out, int max)
{
    if (!g_module || w <= 0 || h <= 0 || w > kSize || h > kSize) return -1;
    const uint32_t t0 = DWT->CYCCNT;

    // Letterbox (keep aspect, grey 114 around), float CHW in [0, 1]. A portrait 240x320
    // camera frame is already 320 tall: scale 1, 40 px of grey each side.
    static float lut[256];
    if (lut[255] == 0.0f) {
        for (int i = 0; i < 256; i++) lut[i] = i / 255.0f;
    }
    const float scale = (float)kSize / (w > h ? w : h);
    const int sw = (int)(w * scale + 0.5f), sh = (int)(h * scale + 0.5f);
    const int ox = (kSize - sw) / 2, oy = (kSize - sh) / 2;
    static int16_t src_x[kSize], src_y[kSize];
    static int tab_w, tab_h;
    if (w != tab_w || h != tab_h) {
        for (int i = 0; i < kSize; i++) {
            int u = i - ox, v = i - oy, sx = (int)(u / scale), sy = (int)(v / scale);
            src_x[i] = (u >= 0 && u < sw) ? (int16_t)(sx < w ? sx : w - 1) : -1;
            src_y[i] = (v >= 0 && v < sh) ? (int16_t)(sy < h ? sy : h - 1) : -1;
        }
        tab_w = w;
        tab_h = h;
    }
    const float grey = lut[114];
    float *pr = g_input, *pg = g_input + kSize * kSize, *pb = g_input + 2 * kSize * kSize;
    for (int y = 0; y < kSize; y++) {
        const int sy = src_y[y];
        for (int x = 0; x < kSize; x++, pr++, pg++, pb++) {
            const int sx = src_x[x];
            if (sy < 0 || sx < 0) {
                *pr = *pg = *pb = grey;
            } else {
                const uint8_t *px = rgb + (sy * w + sx) * 3;
                *pr = lut[px[0]];
                *pg = lut[px[1]];
                *pb = lut[px[2]];
            }
        }
    }

    std::array<SizesType, 4> sizes{1, 3, kSize, kSize};
    std::array<DimOrderType, 4> dim_order{0, 1, 2, 3};
    TensorImpl impl(ScalarType::Float, sizes.size(), sizes.data(), g_input, dim_order.data());
    Tensor input(&impl);
    const uint32_t t1 = DWT->CYCCNT;
    ethos_npu_cycles_take();
    auto outputs = g_module->forward(input);   // copies the input in on every call
    const uint32_t t2 = DWT->CYCCNT;
    g_pre_us = cyc_to_us(t1 - t0);
    g_npu_us = cyc_to_us(t2 - t1);
    g_hw_us = cyc_to_us(ethos_npu_cycles_take());   // the NPU alone; the rest is int8<->float on the CPU
    if (!outputs.ok() || outputs->size() != 3) {
        printf("yolo: forward failed (err=%u)\n", outputs.ok() ? 0u : (unsigned)outputs.error());
        return -1;
    }

    // The three head maps, strides 8, 16, 32: told apart by size, not trusted by order.
    const float *maps[3] = {nullptr, nullptr, nullptr};
    int hw[3] = {40, 20, 10};
    for (size_t i = 0; i < 3; i++) {
        const auto &ev = outputs->at(i);
        if (!ev.isTensor() || ev.toTensor().scalar_type() != ScalarType::Float) return -1;
        const Tensor t = ev.toTensor();
        for (int l = 0; l < 3; l++)
            if (t.numel() == (64 + YOLO_CLASSES) * hw[l] * hw[l]) maps[l] = t.const_data_ptr<float>();
    }
    if (!maps[0] || !maps[1] || !maps[2]) {
        printf("yolo: unexpected output shapes\n");
        return -1;
    }
    waldo::Det dets[16];
    const int kept = waldo::decode(maps, hw, hw, kSize, min_score, kNmsIou, g_scratch, waldo::kMaxCandidates,
                                   dets, 16);
    const int n = kept < max ? (kept < 16 ? kept : 16) : max;
    for (int i = 0; i < n; i++) {   // letterboxed 320 px -> the caller's pixels
        const waldo::Det &d = dets[i];
        out[i] = {(d.x1 - ox) / scale, (d.y1 - oy) / scale, (d.x2 - d.x1) / scale, (d.y2 - d.y1) / scale,
                  d.score, d.cls};
    }
    g_post_us = cyc_to_us(DWT->CYCCNT - t2);
    return n;
}
