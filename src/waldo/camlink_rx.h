// Frames from the ESP32-S3-CAM head over UART0. See camlink_rx.cpp for the wiring.
#pragma once
#include <stdint.h>

#define CAM_MAX_W 160
#define CAM_MAX_H 120

struct cam_frame {
    uint16_t width, height;
    uint8_t seq;
    uint32_t count;   // frames received OK since boot
    uint8_t pix[CAM_MAX_W * CAM_MAX_H];
};

int camlink_init(void);
// Parse whatever has arrived. Call often from the main loop.
void camlink_poll(uint32_t now_ms);
// Newest good frame, or NULL before the first one.
const struct cam_frame *camlink_latest(void);
void camlink_stats(uint32_t now_ms, uint32_t *ok, uint32_t *bad, uint32_t *lost, float *fps);
