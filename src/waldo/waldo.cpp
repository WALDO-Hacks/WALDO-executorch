#include <stdint.h>
#include <stdio.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "waldo.h"
#include "motion.h"
#include "servo_hw.h"
#include "camlink_rx.h"
#include "detector.h"
#include "tracker.h"
#include "face_detector.hpp"

static volatile uint32_t ms;
extern "C" void SysTick_Handler(void) { ms++; }

uint32_t waldo_millis(void) { return ms; }

static void wait_ms(uint32_t n)
{
    uint32_t t0 = ms;
    while (ms - t0 < n) waldo_poll();
}

int waldo_init(void)
{
    SysTick_Config(SystemCoreClock / 1000U);
    if (servo_hw_init() != 0) return -1;
    motion_init(servo_hw_write_us, -1, -1);
    if (camlink_init() != 0) printf("WALDO: camera link (UART0) init failed\n");
    printf("WALDO: servos up, parked (pan %.0f, tilt %.0f)\n",
           (double)motion_pos(AXIS_PAN), (double)motion_pos(AXIS_TILT));
    return 0;
}

void waldo_poll(void)
{
    static uint32_t last, last_stats;
    uint32_t now = ms;
    camlink_poll(now);

    // New frame -> detect -> steer.
    static uint32_t frame_seen;
    const struct cam_frame *f = camlink_latest();
    if (f && f->count != frame_seen) {
        frame_seen = f->count;
        struct det_box boxes[8];
        int n = detect(f, boxes, 8);
        tracker_update(boxes, n, f->width, f->height, now);
    }
    if (now - last_stats >= 2000) {
        uint32_t ok, bad, lost;
        float fps;
        camlink_stats(now, &ok, &bad, &lost, &fps);
        uint32_t mean = 0;
        if (f) {
            for (uint32_t i = 0; i < (uint32_t)f->width * f->height; i++) mean += f->pix[i];
            mean /= (uint32_t)f->width * f->height;
        }
        printf("camera link: %lu ok, %lu bad, %lu lost chunks, %.1f fps, brightness %lu\n",
               (unsigned long)ok, (unsigned long)bad, (unsigned long)lost, (double)fps, (unsigned long)mean);
        tracker_print();
        printf("face: NPU inference %lu us\n", (unsigned long)face_detector_last_us());
        last_stats = now;
    }
    if (now - last >= 20) {
        motion_tick((now - last) / 1000.0f);
        last = now;
    }
}

void waldo_servo_test(void)
{
    static const float steps[][2] = {{30, -90}, {0, -90}, {0, 0}, {0, -90}};
    for (auto &s : steps) {
        printf("WALDO: aim pan %.0f tilt %.0f\n", (double)s[0], (double)s[1]);
        motion_set(AXIS_PAN, s[0]);
        motion_set(AXIS_TILT, s[1]);
        wait_ms(1500);
    }
}

void waldo_home(void)
{
    motion_set(AXIS_PAN, 0);
    motion_set(AXIS_TILT, 0);
}
