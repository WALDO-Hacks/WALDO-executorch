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
#include "board_display.h"
#include "mic.h"
#include "kws.hpp"
#include "orchestrator.h"
#include "view.h"

// Per-stage timing (DWT cycle counter, started by kws_init / yolo_init): mean and worst
// over each 2 s stats window, printed with the stats.
enum { ST_LINK, ST_PRE, ST_NPU, ST_POST, ST_TRACK, ST_DISPLAY, ST_FRAME, ST_N };
static const char *const st_name[ST_N] = {"camera rx", "preprocess", "NPU", "decode+NMS",
                                          "orchestrator+tracker", "display", "frame total"};
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
        int n;
        orch_frame(f, now, boxes, 8, &n);   // runs the detector only when the state needs it
        const uint32_t orch_us = (cycles() - cf) / (SystemCoreClock / 1000000u);
        if (n >= 0) {
            uint32_t pre, npu, post;
            detect_stage_us(&pre, &npu, &post);
            st_add(ST_PRE, pre);
            st_add(ST_NPU, npu);
            st_add(ST_POST, post);
            st_add(ST_TRACK, orch_us - pre - npu - post);
        } else {
            st_add(ST_TRACK, orch_us);
        }
        uint32_t c = cycles();

        waldo_view_draw(f, boxes, n, tracker_target_index(), orch_state());
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
        printf("camera link: %lu ok, %lu bad, %lu lost chunks, %.1f fps, brightness %lu, %s %ux%u, JPEG decode %lu us\n",
               (unsigned long)ok, (unsigned long)bad, (unsigned long)lost, (double)fps, (unsigned long)mean,
               f && f->rgb ? "colour" : "gray", f ? f->width : 0, f ? f->height : 0,
               (unsigned long)camlink_decode_us());
        orch_print(now);
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
    if (word >= 0) {
        printf("HEARD: %s\n", kws_label(word));
        orch_word(kws_label(word), now);
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
    orch_init(ms);   // start in SENTRY, looking ahead
}
