// Face detection with BlazeFace on the Ethos-U85 through ExecuTorch (model/model.py).
#pragma once
#include <stdint.h>

struct face_box {
    float x, y, w, h;   // pixels of the image passed in
    float score;        // 0..1
};

// Loads the .pte into the ExecuTorch runtime. Call once, after runtime_init().
bool face_detector_init();

// Finds faces in a grayscale image of any size up to 1024 px a side: letterboxed into the
// model's 128x128 input, run on the NPU, decoded and de-duplicated on the CPU.
// Returns the number of boxes written to out (at most max), best first; -1 on error.
int face_detect_gray(const uint8_t *img, int w, int h, face_box *out, int max);

// Microseconds the last NPU inference took (forward() only).
uint32_t face_detector_last_us();
