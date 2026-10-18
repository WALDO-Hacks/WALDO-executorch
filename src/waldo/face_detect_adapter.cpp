// WALDO's detect(): YOLOv8n (people + 27 object classes) on the head's colour frames, on
// the Ethos-U85 (src/yolo_detector.cpp). Grayscale frames carry no colour: no detections.
#include "detector.h"
#include "yolo_detector.hpp"

static constexpr float kYoloMinScore = 0.35f;

const char *detect_class_name(int cls) { return yolo_class_name(cls); }

int detect(const struct cam_frame *f, struct det_box *out, int max)
{
    if (!f->rgb) return 0;
    yolo_box b[8];
    int n = yolo_detect_rgb(f->rgb, f->rgb_w, f->rgb_h, kYoloMinScore, b, max < 8 ? max : 8);
    // Colour frames are twice the grayscale frame's size.
    const float kx = (float)f->width / f->rgb_w, ky = (float)f->height / f->rgb_h;
    for (int i = 0; i < n; i++)
        out[i] = {(int)(b[i].x * kx), (int)(b[i].y * ky), (int)(b[i].w * kx), (int)(b[i].h * ky), b[i].score, b[i].cls};
    return n < 0 ? 0 : n;
}

void detect_stage_us(uint32_t *pre, uint32_t *npu, uint32_t *post) { yolo_stage_us(pre, npu, post); }
uint32_t detect_npu_hw_us(void) { return yolo_npu_hw_us(); }
