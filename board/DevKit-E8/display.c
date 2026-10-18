/*---------------------------------------------------------------------------
 * Copyright (c) 2026 Arm Limited (or its affiliates). All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * DevKit-E8 display: shows an RGB image on the 4.3" MIPI-DSI panel (ILI9806E,
 * 480 x 800 portrait) through the CDC200 display controller. The application
 * calls board_display_image() with an 8-bit RGB image (HWC); the image is
 * scaled up by the largest integer factor that fits the panel, centred on a
 * black background and written into the RGB888 frame buffer in bulk SRAM.
 * The display is brought up on the first call (CDC200 + DSI + panel), as in
 * the pack's Boards/Templates/Baremetal/demo_cdc200.c.
 *---------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "RTE_Components.h"
#include CMSIS_device_header
#include "RTE_Device.h"

#include "Driver_CDC200.h"
#include "font6x11.h"

#define LCD_WIDTH   RTE_PANEL_HACTIVE_TIME   /* 480 */
#define LCD_HEIGHT  RTE_PANEL_VACTIVE_LINE   /* 800 */

#if RTE_CDC200_PIXEL_FORMAT != 1
#error "display.c expects RTE_CDC200_PIXEL_FORMAT 1 (RGB888) in RTE_Device.h"
#endif
#define LCD_BPP     3

extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

/* RGB888 frame buffer, 480 x 800 x 3 = 1152000 bytes, in bulk SRAM (NPU and
   display DMA accessible); the layer's linker scripts route the section. */
#ifdef WALDO_NPU_SRAM0_ONLY   /* SRAM0 holds both of YOLO's NPU pools: the frame goes to SRAM1 */
#define LCD_FRAME_SECTION ".bss.sram1.lcd_frame"
#else
#define LCD_FRAME_SECTION ".bss.lcd_frame_buf"
#endif
static uint8_t lcd_frame[LCD_HEIGHT][LCD_WIDTH][LCD_BPP]
    __attribute__((section(LCD_FRAME_SECTION), aligned(32)));

static int display_ready = 0;

/* Where the last image landed on the panel (WALDO: for drawing boxes over it). */
static int img_scale = 1, img_x0, img_y0, img_w, img_h;

static void display_callback(uint32_t event)
{
    if (event & ARM_CDC_DSI_ERROR_EVENT) {
        printf("Display: DSI error event\n");
    }
}

/* Bring up the display controller, DSI link and panel; returns 0 on success. */
static int display_init(void)
{
    int32_t ret;

    memset(lcd_frame, 0, sizeof(lcd_frame));
    SCB_CleanDCache_by_Addr((uint32_t *) lcd_frame, sizeof(lcd_frame));

    ret = CDCdrv->Initialize(display_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("Display: CDC200 Initialize failed (%d)\n", (int) ret);
        return -1;
    }
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("Display: CDC200 PowerControl failed (%d)\n", (int) ret);
        return -1;
    }
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t) lcd_frame);
    if (ret != ARM_DRIVER_OK) {
        printf("Display: CDC200 configure failed (%d)\n", (int) ret);
        return -1;
    }
    ret = CDCdrv->Start();
    if (ret != ARM_DRIVER_OK) {
        printf("Display: CDC200 Start failed (%d)\n", (int) ret);
        return -1;
    }
    printf("Display: %ux%u RGB888 panel started\n", (unsigned) LCD_WIDTH, (unsigned) LCD_HEIGHT);
    return 0;
}

/*
  Show an 8-bit RGB (HWC, or gray when channels == 1) image on the panel.

  \param[in]   rgb       Pixel data, height x width x channels bytes
  \param[in]   width     Image width in pixels
  \param[in]   height    Image height in pixels
  \param[in]   channels  1 (gray) or 3 (RGB)
  \return      0 on success, -1 when the display is not available
*/
int board_display_image(const uint8_t *rgb, int width, int height, int channels)
{
    if (!display_ready) {
        if (display_init() != 0) {
            return -1;
        }
        display_ready = 1;
    }
    if (width <= 0 || height <= 0 || (channels != 1 && channels != 3)) {
        return -1;
    }

    /* Largest integer scale that fits, centred. */
    int scale = LCD_WIDTH / width;
    if (LCD_HEIGHT / height < scale) {
        scale = LCD_HEIGHT / height;
    }
    if (scale < 1) {
        scale = 1;
    }
    const int out_w = width * scale, out_h = height * scale;
    const int x0 = (LCD_WIDTH - out_w) / 2, y0 = (LCD_HEIGHT - out_h) / 2;
    img_scale = scale; img_x0 = x0; img_y0 = y0; img_w = width; img_h = height;

    for (int y = 0; y < out_h && y0 + y < LCD_HEIGHT; y++) {
        const uint8_t *row = rgb + (size_t)(y / scale) * width * channels;
        uint8_t *dst = &lcd_frame[y0 + y][x0][0];
        for (int x = 0; x < out_w && x0 + x < LCD_WIDTH; x++) {
            const uint8_t *px = row + (x / scale) * channels;
            /* CDC200 RGB888: little-endian 0x00RRGGBB in memory, i.e. B, G, R */
            dst[0] = channels == 3 ? px[2] : px[0];
            dst[1] = channels == 3 ? px[1] : px[0];
            dst[2] = px[0];
            dst += LCD_BPP;
        }
    }
    /* The frame buffer is read by the display controller's DMA: push the
       cached lines out (bulk SRAM is write-through, but be explicit). */
    SCB_CleanDCache_by_Addr((uint32_t *) &lcd_frame[y0][0][0], (int32_t)(out_h * LCD_WIDTH * LCD_BPP));
    return 0;
}

/*
  WALDO: outline a box over the last image shown, in that image's pixel coordinates.

  \param[in]   x, y, w, h  Box in image pixels (clipped to the image)
  \param[in]   r, g, b     Colour
  \param[in]   thick       Line width in panel pixels
*/
void board_display_box(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b, int thick)
{
    if (!display_ready || w <= 0 || h <= 0) {
        return;
    }
    int px0 = img_x0 + x * img_scale, py0 = img_y0 + y * img_scale;
    int px1 = px0 + w * img_scale - 1, py1 = py0 + h * img_scale - 1;
    const int lx = img_x0, ly = img_y0, hx = img_x0 + img_w * img_scale - 1, hy = img_y0 + img_h * img_scale - 1;
    if (px0 < lx) px0 = lx;
    if (py0 < ly) py0 = ly;
    if (px1 > hx) px1 = hx;
    if (py1 > hy) py1 = hy;
    if (px0 > px1 || py0 > py1) {
        return;
    }
    for (int yy = py0; yy <= py1; yy++) {
        for (int xx = px0; xx <= px1; xx++) {
            if (yy - py0 < thick || py1 - yy < thick || xx - px0 < thick || px1 - xx < thick) {
                uint8_t *d = &lcd_frame[yy][xx][0];
                d[0] = b; d[1] = g; d[2] = r;
            }
        }
    }
    SCB_CleanDCache_by_Addr((uint32_t *) &lcd_frame[py0][0][0], (int32_t)((py1 - py0 + 1) * LCD_WIDTH * LCD_BPP));
}

/*
  WALDO: fill a rectangle of the panel (panel pixels, clipped).
*/
void board_display_fill(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
    if (!display_ready) {
        return;
    }
    int x1 = x + w, y1 = y + h;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x1 > LCD_WIDTH) x1 = LCD_WIDTH;
    if (y1 > LCD_HEIGHT) y1 = LCD_HEIGHT;
    if (x >= x1 || y >= y1) {
        return;
    }
    for (int yy = y; yy < y1; yy++) {
        for (int xx = x; xx < x1; xx++) {
            uint8_t *d = &lcd_frame[yy][xx][0];
            d[0] = b; d[1] = g; d[2] = r;
        }
    }
    SCB_CleanDCache_by_Addr((uint32_t *) &lcd_frame[y][0][0], (int32_t)((y1 - y) * LCD_WIDTH * LCD_BPP));
}

/*
  WALDO: write ASCII text at panel pixel (x, y), top left, in a 6x11 font scaled up by
  scale; pixels outside the glyphs are left alone. Returns the x after the last character.
*/
int board_display_text(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b, int scale)
{
    if (!display_ready || scale < 1) {
        return x;
    }
    const int y_end = y + FONT_H * scale;
    for (; *text; text++, x += FONT_W * scale) {
        const unsigned c = (unsigned char) *text;
        if (c < 32 || c > 126 || x + FONT_W * scale > LCD_WIDTH) {
            continue;
        }
        for (int row = 0; row < FONT_H; row++) {
            const uint8_t bits = font6x11[c - 32][row];
            for (int col = 0; col < FONT_W; col++) {
                if (!(bits >> (FONT_W - 1 - col) & 1)) {
                    continue;
                }
                for (int dy = 0; dy < scale; dy++) {
                    const int yy = y + row * scale + dy;
                    if (yy < 0 || yy >= LCD_HEIGHT) {
                        continue;
                    }
                    uint8_t *d = &lcd_frame[yy][x + col * scale][0];
                    for (int dx = 0; dx < scale; dx++, d += LCD_BPP) {
                        d[0] = b; d[1] = g; d[2] = r;
                    }
                }
            }
        }
    }
    if (y < 0) y = 0;
    if (y < LCD_HEIGHT) {
        SCB_CleanDCache_by_Addr((uint32_t *) &lcd_frame[y][0][0],
                                (int32_t)(((y_end < LCD_HEIGHT ? y_end : LCD_HEIGHT) - y) * LCD_WIDTH * LCD_BPP));
    }
    return x;
}

/* A label in image pixels (like board_display_box): above (x, y) on a dark strip, or just
   below y if that would leave the picture. Kept inside the picture horizontally. */
void board_display_image_label(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b, int scale)
{
    if (!display_ready || !text || !*text) {
        return;
    }
    int n = 0;
    while (text[n]) n++;
    const int tw = n * FONT_W * scale + 4, th = FONT_H * scale + 2;
    int px = img_x0 + x * img_scale, py = img_y0 + y * img_scale - th;
    if (py < img_y0) py = img_y0 + y * img_scale + 2;   /* no room above: inside the box */
    if (py < img_y0) py = img_y0;
    const int right = img_x0 + img_w * img_scale;
    if (px + tw > right) px = right - tw;
    if (px < img_x0) px = img_x0;
    board_display_fill(px, py, tw, th, 0, 0, 0);
    board_display_text(px + 2, py + 1, text, r, g, b, scale);
}
