// From the WALDO team's vision hand-off (handoff/waldo_yolo/include), unchanged.
// waldo_yolo_decode.h -- CPU post-processing for the WALDO YOLOv8n int8 model (header-only, C++11, only <math.h>/<stdint.h>).
//
// The NPU graph returns three raw head maps (no DFL, no sigmoid, no NMS). This file does: DFL -> box, sigmoid(class logit),
// confidence threshold, class-aware greedy NMS. It allocates nothing and touches no hardware: the caller passes the scratch
// buffer, so YOU decide which memory it lives in. Reference implementation in Python: host/export/yolo_wrapper.py::decode().
//
// Tensor layout (float32, as produced by the .pte's dequantized outputs), for each of the 3 maps l = 0,1,2:
//     maps[l][c * H * W + y * W + x],  c in [0, 64 + kNumClasses)
//     c in [0, 64)    : box distances, 4 sides (left, top, right, bottom) x 16 DFL bins; side k bin j at channel k*16 + j
//     c in [64, 64+NC): class logits (sigmoid applied here), order = waldo::kClassNames
//     map l has H = W = imgsz / stride_l with strides 8, 16, 32 (for imgsz 320: 40, 20, 10)
// Output boxes are xyxy in pixels of the *letterboxed* model input (imgsz x imgsz). Undo your letterbox to get camera pixels.
#pragma once
#include <math.h>
#include <stdint.h>

namespace waldo {

constexpr int kNumClasses = 28;     // must match the .pte (see MODEL_CARD.md); waldo_yolo_classes.h has the names
constexpr int kRegMax = 16;         // DFL bins per box side
constexpr int kMaxCandidates = 2200;  // >= 40*40+20*20+10*10 anchors at imgsz 320, so nothing is ever dropped

struct Det { float x1, y1, x2, y2, score; int32_t cls; };  // 24 bytes

static inline float iou(const Det& a, const Det& b) {
    const float ix1 = fmaxf(a.x1, b.x1), iy1 = fmaxf(a.y1, b.y1), ix2 = fminf(a.x2, b.x2), iy2 = fminf(a.y2, b.y2);
    const float iw = fmaxf(0.f, ix2 - ix1), ih = fmaxf(0.f, iy2 - iy1), inter = iw * ih;
    const float u = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
    return u > 0.f ? inter / u : 0.f;
}

// scratch: caller-owned array of >= scratch_cap Det (use kMaxCandidates). out: caller-owned array of out_cap Det.
// Returns the number of detections kept after NMS (may exceed out_cap; only the first out_cap, highest score first, are written).
// *dropped (optional) = candidates that did not fit in scratch (0 with scratch_cap >= kMaxCandidates at imgsz 320).
static inline int decode(const float* const maps[3], const int h[3], const int w[3], int imgsz, float conf, float iou_thr,
                         Det* scratch, int scratch_cap, Det* out, int out_cap, int* dropped = nullptr) {
    int n = 0, overflow = 0;
    for (int l = 0; l < 3; l++) {
        const int H = h[l], W = w[l], HW = H * W;
        const float stride = (float)imgsz / (float)H;
        const float* m = maps[l];
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                const int p = y * W + x;
                float best = -1e30f; int bc = 0;
                for (int c = 0; c < kNumClasses; c++) {
                    const float v = m[(4 * kRegMax + c) * HW + p];
                    if (v > best) { best = v; bc = c; }
                }
                const float score = 1.0f / (1.0f + expf(-best));
                if (score <= conf) continue;
                float d[4];
                for (int k = 0; k < 4; k++) {
                    float mx = -1e30f;
                    for (int j = 0; j < kRegMax; j++) mx = fmaxf(mx, m[(k * kRegMax + j) * HW + p]);
                    float sum = 0.f, ex = 0.f;
                    for (int j = 0; j < kRegMax; j++) {
                        const float e = expf(m[(k * kRegMax + j) * HW + p] - mx);
                        sum += e; ex += e * (float)j;
                    }
                    d[k] = ex / sum;
                }
                const float cx = (float)x + 0.5f, cy = (float)y + 0.5f;
                if (n < scratch_cap) scratch[n++] = {(cx - d[0]) * stride, (cy - d[1]) * stride, (cx + d[2]) * stride, (cy + d[3]) * stride, score, bc};
                else overflow++;
            }
        }
    }
    if (dropped) *dropped = overflow;
    // class-aware greedy NMS, highest score first
    bool dead[kMaxCandidates] = {false};
    if (n > kMaxCandidates) n = kMaxCandidates;
    int kept = 0;
    for (;;) {
        int bi = -1;
        for (int i = 0; i < n; i++) if (!dead[i] && (bi < 0 || scratch[i].score > scratch[bi].score)) bi = i;
        if (bi < 0) break;
        const Det d = scratch[bi];
        dead[bi] = true;
        for (int i = 0; i < n; i++) if (!dead[i] && scratch[i].cls == d.cls && iou(d, scratch[i]) > iou_thr) dead[i] = true;
        if (kept < out_cap) out[kept] = d;
        kept++;
    }
    return kept;
}

}  // namespace waldo
