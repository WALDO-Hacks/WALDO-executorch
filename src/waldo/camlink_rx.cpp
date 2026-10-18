// Camera link receiver: frames from the ESP32-S3-CAM head over UART0 (camlink.h).
//   E8 UART0 RX = P1_4 = J10 "CK_TX" (pin 33)  <- ESP32 GPIO14
//   E8 UART0 TX = P1_5 = J10 "CK_RX" (pin 35)  -> ESP32 GPIO21
// The USART driver fills 512-byte chunks from its interrupt; each finished chunk goes
// into a ring buffer and the next receive starts at once. camlink_poll(), called from
// the main loop, parses frames out of the ring.

#include "camlink_rx.h"

#include <stdio.h>
#include <string.h>

#include "Driver_USART.h"
#include "pinconf.h"
#include "camlink.h"
#include "jpeg_frame.h"

#include "RTE_Components.h"
#include CMSIS_device_header

extern ARM_DRIVER_USART Driver_USART0;
static ARM_DRIVER_USART *const usart = &Driver_USART0;

// ---------- ring buffer (ISR writes, main loop reads) ----------

#define RING_SIZE 65536u   // power of two
static uint8_t ring[RING_SIZE];
static volatile uint32_t ring_head, ring_tail;
static volatile uint32_t overruns;

static void ring_put(const uint8_t *d, uint32_t n)
{
    uint32_t head = ring_head;
    if (RING_SIZE - (head - ring_tail) < n) {
        overruns++;
        return;
    }
    for (uint32_t i = 0; i < n; i++) ring[(head + i) & (RING_SIZE - 1)] = d[i];
    ring_head = head + n;
}

// ---------- USART chunks ----------

#define CHUNK 512u
static uint8_t chunk[2][CHUNK];
static volatile int active;

static void usart_event(uint32_t event)
{
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        int done = active;
        active ^= 1;
        usart->Receive(chunk[active], CHUNK);
        ring_put(chunk[done], CHUNK);
    }
    if (event & ARM_USART_EVENT_RX_OVERFLOW) overruns++;
}

// ---------- frame parser ----------

static enum { HUNT, HEADER, PAYLOAD, CHECKSUM } state = HUNT;
static camlink_hdr_t hdr;
static uint32_t got;
#define MAX_PAYLOAD 32768u   // a 320x240 JPEG is ~8-15 KB; a raw grayscale frame 19.2 KB
static uint8_t staging[MAX_PAYLOAD];
// Decoded colour frame and the rotation scratch, in SRAM1 (230 KB each).
static uint8_t rgb_buf[JPEG_MAX_W * JPEG_MAX_H * 3] __attribute__((section(".bss.sram1.cam_rgb"), aligned(32)));
static uint8_t rot_buf[JPEG_MAX_W * JPEG_MAX_H * 3] __attribute__((section(".bss.sram1.cam_rot"), aligned(32)));
static uint8_t gray_buf[CAM_MAX_W * CAM_MAX_H];
static uint32_t decode_us;
static uint8_t sum_bytes[2];

static struct cam_frame latest;
static uint32_t n_ok, n_bad, first_ok_ms, last_ok_ms;

// Rotates src (w x h, bpp bytes per pixel) clockwise by WALDO_CAM_ROTATE into dst and
// updates w, h. With 0 it is a copy (or nothing if src == dst); for colour frames dst is
// scratch and the result is copied back into src.
static void rotate(uint8_t *src, uint8_t *dst, uint16_t w, uint16_t h, int bpp, uint16_t *ow, uint16_t *oh)
{
    const int rot = WALDO_CAM_ROTATE;
    const bool in_place = bpp == 3;   // colour: rotate into scratch, copy back
    uint8_t *out = dst;
    const uint32_t n = (uint32_t)w * h * bpp;
    if (rot == 0) {
        if (!in_place && src != dst) memcpy(dst, src, n);
        *ow = w;
        *oh = h;
        return;
    }
    const uint16_t nw = (rot == 180) ? w : h, nh = (rot == 180) ? h : w;
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            uint32_t nx, ny;
            if (rot == 90) { nx = h - 1 - y; ny = x; }
            else if (rot == 180) { nx = w - 1 - x; ny = h - 1 - y; }
            else { nx = y; ny = w - 1 - x; }   // 270
            memcpy(out + (ny * nw + nx) * bpp, src + (y * w + x) * bpp, bpp);
        }
    }
    if (in_place) memcpy(src, out, n);
    *ow = nw;
    *oh = nh;
}

static void frame_done(uint32_t now_ms)
{
    uint16_t sum = (uint16_t)(sum_bytes[0] | (sum_bytes[1] << 8));
    if (sum != camlink_fletcher16(staging, hdr.len)) {
        n_bad++;
        return;
    }
    uint16_t w = hdr.width, h = hdr.height;
    if (hdr.format == CAMLINK_FMT_JPEG) {
        const uint32_t t0 = DWT->CYCCNT;
        uint16_t jw, jh;
        if (jpeg_frame_decode(staging, hdr.len, rgb_buf, gray_buf, &jw, &jh) != 0 ||
            jw / 2 > CAM_MAX_W || jh / 2 > CAM_MAX_H) {
            n_bad++;
            return;
        }
        latest.rgb_w = jw;
        latest.rgb_h = jh;
        rotate(rgb_buf, rot_buf, jw, jh, 3, &latest.rgb_w, &latest.rgb_h);
        latest.rgb = rgb_buf;
        w = jw / 2;
        h = jh / 2;
        rotate(gray_buf, latest.pix, w, h, 1, &w, &h);
        decode_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
    } else {
        rotate(staging, latest.pix, w, h, 1, &w, &h);
        latest.rgb = NULL;
        decode_us = 0;
    }
    if (!n_ok) first_ok_ms = now_ms;
    last_ok_ms = now_ms;
    n_ok++;
    latest.width = w;
    latest.height = h;
    latest.seq = hdr.seq;
    latest.count = n_ok;
}

void camlink_poll(uint32_t now_ms)
{
    static const char magic[4] = {'W', 'L', 'D', 'F'};
    while (ring_tail != ring_head) {
        uint8_t b = ring[ring_tail & (RING_SIZE - 1)];
        ring_tail = ring_tail + 1;
        switch (state) {
        case HUNT:
            ((uint8_t *)&hdr)[got] = b;
            got = (b == (uint8_t)magic[got]) ? got + 1 : (b == (uint8_t)magic[0]);
            if (got == 4) state = HEADER;
            break;
        case HEADER:
            ((uint8_t *)&hdr)[got++] = b;
            if (got == sizeof hdr) {
                got = 0;
                const bool gray_ok = hdr.format == CAMLINK_FMT_GRAY8 && hdr.width <= CAM_MAX_W &&
                                     hdr.height <= CAM_MAX_H && hdr.len == (uint32_t)hdr.width * hdr.height;
                const bool jpeg_ok = hdr.format == CAMLINK_FMT_JPEG && hdr.width <= JPEG_MAX_W &&
                                     hdr.height <= JPEG_MAX_H && hdr.len > 0 && hdr.len <= MAX_PAYLOAD;
                if (!gray_ok && !jpeg_ok) {
                    n_bad++;
                    state = HUNT;
                } else {
                    state = PAYLOAD;
                }
            }
            break;
        case PAYLOAD:
            staging[got++] = b;
            if (got == hdr.len) {
                got = 0;
                state = CHECKSUM;
            }
            break;
        case CHECKSUM:
            sum_bytes[got++] = b;
            if (got == 2) {
                frame_done(now_ms);
                got = 0;
                state = HUNT;
            }
            break;
        }
    }
}

// ---------- API ----------

int camlink_init(void)
{
    // P1_4 = UART0_RX_B, P1_5 = UART0_TX_B (alternate function 2).
    pinconf_set(PORT_1, PIN_4, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_READ_ENABLE);
    pinconf_set(PORT_1, PIN_5, PINMUX_ALTERNATE_FUNCTION_2, 0);

    if (usart->Initialize(usart_event) != ARM_DRIVER_OK) return -1;
    if (usart->PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK) return -1;
    if (usart->Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE |
                           ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE,
                       CAMLINK_BAUD) != ARM_DRIVER_OK) return -1;
    if (usart->Control(ARM_USART_CONTROL_RX, 1) != ARM_DRIVER_OK) return -1;
    active = 0;
    if (usart->Receive(chunk[0], CHUNK) != ARM_DRIVER_OK) return -1;
    return 0;
}

uint32_t camlink_decode_us(void) { return decode_us; }

const struct cam_frame *camlink_latest(void)
{
    return latest.count ? &latest : NULL;
}

void camlink_stats(uint32_t now_ms, uint32_t *ok, uint32_t *bad, uint32_t *lost, float *fps)
{
    *ok = n_ok;
    *bad = n_bad;
    *lost = overruns;
    uint32_t dt = last_ok_ms - first_ok_ms;
    *fps = (n_ok > 1 && dt > 0) ? (n_ok - 1) * 1000.0f / dt : 0;
    (void)now_ms;
}
