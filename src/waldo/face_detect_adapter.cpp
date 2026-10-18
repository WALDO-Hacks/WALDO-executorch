// WALDO's detect(): YOLOv8n (people + 27 object classes) on colour frames, BlazeFace on
// grayscale ones. Both run on the Ethos-U85 (src/yolo_detector.cpp, src/face_detector.cpp).
#include "detector.h"
#include "face_detector.hpp"
#include "yolo_detector.hpp"

static bool use_faces;
static bool last_was_yolo;
static constexpr float kYoloMinScore = 0.35f;

void detect_use_faces(bool faces) { use_faces = faces; }

const char *detect_class_name(int cls) { return cls == DET_FACE ? "face" : yolo_class_name(cls); }

int detect(const struct cam_frame *f, struct det_box *out, int max)
{
    if (f->rgb && !use_faces) {
        yolo_box b[8];
        int n = yolo_detect_rgb(f->rgb, f->rgb_w, f->rgb_h, kYoloMinScore, b, max < 8 ? max : 8);
        last_was_yolo = true;
        // Colour frames are twice the grayscale frame's size.
        const float kx = (float)f->width / f->rgb_w, ky = (float)f->height / f->rgb_h;
        for (int i = 0; i < n; i++)
            out[i] = {(int)(b[i].x * kx), (int)(b[i].y * ky), (int)(b[i].w * kx), (int)(b[i].h * ky), b[i].score, b[i].cls};
        return n < 0 ? 0 : n;
    }
    face_box faces[8];
    int n = face_detect_gray(f->pix, f->width, f->height, faces, max < 8 ? max : 8);
    last_was_yolo = false;
    for (int i = 0; i < n; i++)
        out[i] = {(int)faces[i].x, (int)faces[i].y, (int)faces[i].w, (int)faces[i].h, faces[i].score, DET_FACE};
    return n < 0 ? 0 : n;
}

void detect_stage_us(uint32_t *pre, uint32_t *npu, uint32_t *post)
{
    if (last_was_yolo) yolo_stage_us(pre, npu, post);
    else face_detector_stage_us(pre, npu, post);
}
