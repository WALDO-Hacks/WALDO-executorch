// Camera JPEGs from the ESP32 head -> pixels for the models and the screen.
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define JPEG_MAX_W 320
#define JPEG_MAX_H 240
#define JPEG_MAX_SIDE 320   // portrait 240x320 too: at most 320x240 pixels either way

// Decodes a baseline JPEG of at most 320x240 pixels (landscape or portrait) into rgb (RGB888, w*h*3 bytes) and, if gray is
// not NULL, a half-size grayscale copy (w/2 x h/2, 2x2 averaged). 0 on success.
int jpeg_frame_decode(const uint8_t *jpg, uint32_t len, uint8_t *rgb, uint8_t *gray,
                      uint16_t *w, uint16_t *h);

// Rotates src (w x h, bpp bytes per pixel) clockwise by deg (0, 90, 180, 270) into dst
// (must not overlap src) and writes the new size to ow, oh.
void frame_rotate(const uint8_t *src, uint8_t *dst, uint16_t w, uint16_t h, int bpp, int deg,
                  uint16_t *ow, uint16_t *oh);

#ifdef __cplusplus
}
#endif
