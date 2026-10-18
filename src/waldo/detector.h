// What WALDO aims at. Any detector fills boxes in camera-frame pixels; the tracker
// doesn't care which model produced them. Swap bright_detector.cpp for the ExecuTorch
// person/face model by implementing detect() over the same frame.
#pragma once
#include "camlink_rx.h"

struct det_box {
    int x, y, w, h;   // camera-frame pixels (160x120)
    float score;
};

// Returns the number of boxes written (at most max).
int detect(const struct cam_frame *f, struct det_box *out, int max);
