#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "waldo.h"
#include "motion.h"
#include "servo_hw.h"
#include "camlink_rx.h"
#include "detector.h"
#include "tracker.h"
#include "face_detector.hpp"
#include "board_display.h"
#include "mic.h"
#include "kws.hpp"

// Per-stage timing (DWT cycle counter, started by face_detector_init): mean and worst
// over each 2 s stats window, printed with the stats.
enum { ST_LINK, ST_PRE, ST_NPU, ST_POST, ST_TRACK, ST_DISPLAY, ST_FRAME, ST_N };
static const char *const st_name[ST_N] = {"camera rx", "preprocess", "NPU", "decode+NMS",
                                          "tracker", "display", "frame total"};
static struct { uint64_t sum; uint32_t max, n; } st[ST_N];

static inline uint32_t cycles(void) { return DWT->CYCCNT; }
static void st_add(int i, uint32_t us)
{
    st[i].sum += us;
    st[i].n++;
    if (us > st[i].max) st[i].max = us;
}
static void st_add_cyc(int i, uint32_t c) { st_add(i, c / (SystemCoreClock / 1000000u)); }

static void st_print(uint32_t window_ms)
{
    printf("timing (us, mean/max over %lu frames):", (unsigned long)st[ST_FRAME].n);
    for (int i = 0; i < ST_N; i++) {
        if (i == ST_LINK) continue;
        uint32_t mean = st[i].n ? (uint32_t)(st[i].sum / st[i].n) : 0;
        printf(" %s %lu/%lu%s", st_name[i], (unsigned long)mean, (unsigned long)st[i].max,
               i == ST_N - 1 ? "\n" : ",");
    }
    // CPU time spent parsing camera bytes, as a share of the window.
    printf("timing: camera rx %lu us total (%.1f%% CPU), frame work %.1f%% CPU\n",
           (unsigned long)st[ST_LINK].sum, (double)st[ST_LINK].sum / (window_ms * 10.0),
           (double)st[ST_FRAME].sum / (window_ms * 10.0));
    for (auto &x : st) x = {0, 0, 0};
}

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
    if (mic_init() != 0) printf("WALDO: microphones init failed\n");
    printf("WALDO: servos up, parked (pan %.0f, tilt %.0f)\n",
           (double)motion_pos(AXIS_PAN), (double)motion_pos(AXIS_TILT));
    return 0;
}

void waldo_poll(void)
{
    static uint32_t last, last_stats;
    uint32_t now = ms;
    uint32_t c0 = cycles();
    camlink_poll(now);
    st_add_cyc(ST_LINK, cycles() - c0);

    // New frame -> detect -> steer.
    static uint32_t frame_seen;
    const struct cam_frame *f = camlink_latest();
    if (f && f->count != frame_seen) {
        frame_seen = f->count;
        const uint32_t cf = cycles();
        struct det_box boxes[8];
        int n = detect(f, boxes, 8);
        uint32_t pre, npu, post;
        face_detector_stage_us(&pre, &npu, &post);
        st_add(ST_PRE, pre);
        st_add(ST_NPU, npu);
        st_add(ST_POST, post);
        uint32_t c = cycles();
        tracker_update(boxes, n, f->width, f->height, now);
        st_add_cyc(ST_TRACK, cycles() - c);
        c = cycles();

        // What WALDO sees, on the LCD: the camera frame, faces boxed, the tracked one green.
        if (board_display_image(f->pix, f->width, f->height, 1) == 0) {
            for (int i = 0; i < n; i++) {
                bool target = i == tracker_target_index();
                board_display_box(boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h,
                                  target ? 0 : 255, target ? 255 : 200, 0, target ? 3 : 2);
            }
            // Centre mark: the tracker drives the target onto it.
            board_display_box(f->width / 2 - 2, f->height / 2 - 2, 5, 5, 255, 255, 255, 1);
        }
        st_add_cyc(ST_DISPLAY, cycles() - c);
        st_add_cyc(ST_FRAME, cycles() - cf);
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
        st_print(now - last_stats);
        float r0, r1;
        uint32_t errs, mfcc_us, npu_us, windows;
        mic_levels(&r0, &r1, &errs);
        kws_listen_stats(&mfcc_us, &npu_us, &windows);
        printf("voice: mic rms %.0f/%.0f (%lu errors), %lu windows, MFCC %lu us, NPU %lu us total\n",
               (double)r0, (double)r1, (unsigned long)errs, (unsigned long)windows,
               (unsigned long)mfcc_us, (unsigned long)npu_us);
        last_stats = now;
    }
    // Voice: the keyword spotter listens all the time.
    int word = kws_listen(now);
    if (word >= 0) printf("HEARD: %s\n", kws_label(word));

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
