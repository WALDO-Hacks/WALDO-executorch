// People and objects with YOLOv8n (28 classes) on the Ethos-U85 through ExecuTorch
// (model/yolo). The team's vision lead built the model; src/yolo/ has its decoder.
#pragma once
#include <stdint.h>

#define YOLO_CLASSES 28
#define YOLO_PERSON 0

struct yolo_box {
    float x, y, w, h;   // pixels of the image passed in
    float score;        // 0..1
    int cls;            // index into yolo_class_name()
};

// Loads the .pte into the ExecuTorch runtime. Call once, after runtime_init().
bool yolo_init();
// Finds people and objects in an RGB888 image (HWC) of up to 320 px a side: letterboxed
// into the model's 320x320 input, run on the NPU, decoded with NMS on the CPU. Returns
// the number of boxes written (at most max), best first; -1 on error.
int yolo_detect_rgb(const uint8_t *rgb, int w, int h, float min_score, yolo_box *out, int max);
const char *yolo_class_name(int cls);
// The last call split up: letterbox (CPU), forward() (NPU), decode + NMS (CPU), in us.
void yolo_stage_us(uint32_t *pre, uint32_t *npu, uint32_t *post);
// Of the last forward(): the time the NPU itself was busy (the rest is CPU-side I/O conversion).
uint32_t yolo_npu_hw_us();
