// WALDO's detect(): faces from BlazeFace on the Ethos-U85 (src/face_detector.cpp).
#include "detector.h"
#include "face_detector.hpp"

int detect(const struct cam_frame *f, struct det_box *out, int max)
{
    face_box faces[8];
    int n = face_detect_gray(f->pix, f->width, f->height, faces, max < 8 ? max : 8);
    for (int i = 0; i < n; i++) {
        out[i] = {(int)faces[i].x, (int)faces[i].y, (int)faces[i].w, (int)faces[i].h, faces[i].score};
    }
    return n < 0 ? 0 : n;
}
