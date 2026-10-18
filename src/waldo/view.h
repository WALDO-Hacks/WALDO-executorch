// What WALDO sees, on the LCD.
#pragma once
#include "camlink_rx.h"
#include "detector.h"
#include "orchestrator.h"

// The camera frame (colour when the head sends JPEG), the detections (the tracked one
// green while tracking), a centre mark and a bar in the state's colour along the top.
// n < 0: the detector didn't run on this frame, no boxes.
void waldo_view_draw(const struct cam_frame *f, const struct det_box *boxes, int n, int target,
                     enum orch_state state);
