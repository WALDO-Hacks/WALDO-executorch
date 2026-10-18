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

// The status bands above and below the picture (the portrait camera view leaves 80 px of
// the 480x800 panel free at each end): state and what WALDO follows on top; the last word
// it heard, frame rate and NPU time at the bottom. Redrawn only when the text changes.
struct view_status {
    enum orch_state state;
    const char *following;   // class name
    const char *heard;       // last voice command, or NULL
    uint32_t heard_age_ms;
    float fps;               // frames processed per second
    uint32_t npu_us;         // last detector inference
    uint32_t frame_us;       // last frame's total work
};
void waldo_view_status(const struct view_status *s);
