/* DevKit-E8 LCD (display.c, from the hackathon-pico-faces branch, plus box drawing). */
#ifndef BOARD_DISPLAY_H
#define BOARD_DISPLAY_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Show an 8-bit gray (channels 1) or RGB (3) image, scaled up and centred. 0 on success. */
int board_display_image(const uint8_t *rgb, int width, int height, int channels);
/* Outline a box over the last image shown, in that image's pixels. */
void board_display_box(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b, int thick);

#ifdef __cplusplus
}
#endif
#endif
