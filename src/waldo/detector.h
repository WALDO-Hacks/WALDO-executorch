// What WALDO aims at. The detector fills boxes in camera-frame pixels (the grayscale
// frame's, 160x120 or 120x160 rotated); the tracker doesn't care which model made them.
#pragma once
#include <stdint.h>
#include "camlink_rx.h"


struct det_box {
    int x, y, w, h;   // camera-frame pixels
    float score;
    int cls;          // YOLO class index (detect_class_name)
};

// People and objects with YOLOv8n on the head's colour frames. Returns the number of boxes
// (at most max).
int detect(const struct cam_frame *f, struct det_box *out, int max);
// Name of a box's class ("person", "cup", ...).
const char *detect_class_name(int cls);
// The last detect() split up: preprocess, NPU, decode (us); npu = 0 if it didn't run.
void detect_stage_us(uint32_t *pre, uint32_t *npu, uint32_t *post);
// Of that NPU time, how long the NPU itself was busy (the rest: int8 <-> float on the CPU).
uint32_t detect_npu_hw_us(void);
