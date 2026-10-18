// What WALDO aims at. The detector fills boxes in camera-frame pixels (the grayscale
// frame's, 160x120 or 120x160 rotated); the tracker doesn't care which model made them.
#pragma once
#include <stdint.h>
#include "camlink_rx.h"

#define DET_FACE (-1)   // cls of a BlazeFace box; YOLO boxes carry their class index

struct det_box {
    int x, y, w, h;   // camera-frame pixels
    float score;
    int cls;
};

// People and objects with YOLOv8n when the head sends colour frames, faces with BlazeFace
// otherwise (or when detect_use_faces(true)). Returns the number of boxes (at most max).
int detect(const struct cam_frame *f, struct det_box *out, int max);
void detect_use_faces(bool faces);
// Name of a box's class ("face", "person", "cup", ...).
const char *detect_class_name(int cls);
// The last detect() split up: preprocess, NPU, decode (us); npu = 0 if it didn't run.
void detect_stage_us(uint32_t *pre, uint32_t *npu, uint32_t *post);
