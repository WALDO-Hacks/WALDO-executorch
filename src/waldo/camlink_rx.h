// Frames from the ESP32-S3-CAM head over UART0. See camlink_rx.cpp for the wiring.
#pragma once
#include <stdint.h>

#define CAM_MAX_W 160
#define CAM_MAX_H 120

// How the camera head is mounted: frames are rotated clockwise by this many degrees
// (0, 90, 180, 270) so that "up" in the frame is "tilt up" for the turret.
#ifndef WALDO_CAM_ROTATE
#define WALDO_CAM_ROTATE 0
#endif

struct cam_frame {
    uint16_t width, height;   // of pix (after rotation)
    uint8_t seq;
    uint32_t count;           // frames received OK since boot
    uint8_t pix[CAM_MAX_W * CAM_MAX_H];   // grayscale: the face tracker and the screen
    // Colour (JPEG mode only): RGB888, rgb_w x rgb_h (twice pix's size), rotated like pix.
    const uint8_t *rgb;       // NULL in grayscale mode
    uint16_t rgb_w, rgb_h;
};

int camlink_init(void);
// Parse whatever has arrived. Call often from the main loop.
void camlink_poll(uint32_t now_ms);
// Newest good frame, or NULL before the first one.
const struct cam_frame *camlink_latest(void);
void camlink_stats(uint32_t now_ms, uint32_t *ok, uint32_t *bad, uint32_t *lost, float *fps);
// Microseconds the last JPEG decode (+ rotation) took; 0 in grayscale mode.
uint32_t camlink_decode_us(void);
