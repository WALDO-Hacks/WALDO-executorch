#include <stdint.h>
#include <stdio.h>
#include <string.h>
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
#ifdef WALDO_TWO_BRAIN
#include "brain_hp.h"
#endif

// Per-stage timing (DWT cycle counter, started by kws_init / yolo_init): mean and worst
// over each 2 s stats window, printed with the stats.
enum { ST_LINK, ST_PRE, ST_NPU, ST_NPU_HW, ST_POST, ST_TRACK, ST_DISPLAY, ST_FRAME, ST_N };
static const char *const st_name[ST_N] = {"camera rx", "preprocess", "forward", "of which NPU busy", "decode+NMS",
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
static bool orch_running;   // waldo_home() starts the orchestrator; the boot servo test runs first

// For the on-screen status: the last word heard, frames per second, last NPU and frame time.
static const char *heard_word;
static uint32_t heard_ms, last_npu_us, last_frame_us, last_yolo_ms;
static float shown_fps;
static volatile bool servos_running;   // motion_init() done: the tick drives the servos
// The servos move from here at a steady 50 Hz (their pulse rate), not from waldo_poll(): a
// frame takes ~50 ms (YOLO + display), and stepping the ramp only between frames made
// patrol moves stutter 6-10 degrees at a time (board, 2026-10-18).
extern "C" void SysTick_Handler(void)
{
    ms++;
    if (servos_running && ms % 20 == 0) motion_tick(0.020f);
}

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
#ifdef WALDO_SERVO_ZERO
    // Assembly aid: both servos at their 1500 us centre, nothing else runs. Fit the horns so
    // pan points straight ahead and tilt is level, then rebuild without WALDO_SERVO_ZERO.
    // Wiggles +-20 degrees (222 us) every few seconds so you can see and hear both servos
    // respond, and rests at the centre in between.
    printf("WALDO: servo zero build, pan and tilt at 1500 us, wiggling +-20 deg every 4 s\n");
    for (int step = 0;; step++) {
        const float us = (step % 4 == 1) ? 1722.0f : (step % 4 == 3) ? 1278.0f : 1500.0f;
        servo_hw_write_us(0, us);
        // Tilt never below TILT_MIN_DEG (the laser under the flashlight): -20 only if allowed.
        const float tilt_deg = CAL_TILT.sign * (us - 1500.0f) / CAL_TILT.us_per_deg;
        const float tilt_ok = tilt_deg < TILT_MIN_DEG ? TILT_MIN_DEG : tilt_deg;
        servo_hw_write_us(1, 1500.0f + CAL_TILT.sign * tilt_ok * CAL_TILT.us_per_deg);
        printf("WALDO: servo zero: %s\n", us > 1500 ? "+20" : us < 1500 ? "-20" : "centre");
        const uint32_t t = ms;
        while (ms - t < (us == 1500.0f ? 3000u : 1000u)) __WFI();
    }
#endif
    motion_init(servo_hw_write_us, -1, -1);
    servos_running = true;
    if (camlink_init() != 0) printf("WALDO: camera link (UART0) init failed\n");
#ifdef WALDO_TWO_BRAIN
    printf("WALDO: two-brain build, the M55-HE owns the microphones\n");
#else
    if (mic_init() != 0) printf("WALDO: microphones init failed\n");
#endif
    printf("WALDO: servos up, parked (pan %.0f, tilt %.0f)\n",
           (double)motion_pos(AXIS_PAN), (double)motion_pos(AXIS_TILT));
    return 0;
}

void waldo_poll(void)
{
    static uint32_t last_stats;
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
        n = -1;
        if (orch_running) orch_frame(f, now, boxes, 8, &n);   // runs the detector only when the state needs it
        const uint32_t orch_us = (cycles() - cf) / (SystemCoreClock / 1000000u);
        if (n >= 0) {
            uint32_t pre, npu, post;
            detect_stage_us(&pre, &npu, &post);
            st_add(ST_PRE, pre);
            st_add(ST_NPU, npu);
            st_add(ST_NPU_HW, detect_npu_hw_us());
            last_npu_us = npu;
            st_add(ST_POST, post);
            st_add(ST_TRACK, orch_us - pre - npu - post);
        } else {
            st_add(ST_TRACK, orch_us);
        }
        uint32_t c = cycles();

        waldo_view_draw(f, boxes, n, tracker_target_index(), orch_state());
        st_add_cyc(ST_DISPLAY, cycles() - c);
        last_frame_us = (cycles() - cf) / (SystemCoreClock / 1000000u);
        st_add(ST_FRAME, last_frame_us);

        // Frames per second over 1 s windows, and the status bands around the picture.
        static uint32_t fps_frames, fps_t0;
        fps_frames++;
        if (now - fps_t0 >= 1000) {
            shown_fps = fps_frames * 1000.0f / (now - fps_t0);
            fps_frames = 0;
            fps_t0 = now;
        }
        const char *detail = orch_detail();   // "scan 3/10" while patrolling
        if (n >= 0) last_yolo_ms = now;
        bool ears_ok = false;
        float ears_busy = 0;
        bool two_brain = false;
#ifdef WALDO_TWO_BRAIN
        two_brain = true;
        ears_ok = brain_hp_status(now, &ears_busy);
#endif
        const struct view_status vs = {orch_state(), detail ? detail : detect_class_name(tracker_target_class()), heard_word,
                                       now - heard_ms, shown_fps, last_npu_us, last_frame_us,
                                       now - last_yolo_ms < 500, two_brain, ears_ok, ears_busy};
        waldo_view_status(&vs);
    }
    if (now - last_stats >= 2000) {
        uint32_t ok, bad, lost;
        float fps;
        camlink_stats(now, &ok, &bad, &lost, &fps);
        uint32_t mean = 0;
        if (f && f->width && f->height) {
            for (uint32_t i = 0; i < (uint32_t)f->width * f->height; i++) mean += f->pix[i];
            mean /= (uint32_t)f->width * f->height;
        }
        printf("camera link: %lu ok, %lu bad, %lu lost chunks, %.1f fps, brightness %lu, %s %ux%u, JPEG decode %lu us, %lu skipped\n",
               (unsigned long)ok, (unsigned long)bad, (unsigned long)lost, (double)fps, (unsigned long)mean,
               f && f->rgb ? "colour" : "gray", f ? f->width : 0, f ? f->height : 0,
               (unsigned long)camlink_decode_us(), (unsigned long)camlink_skipped());
        orch_print(now);
        tracker_print();
        st_print(now - last_stats);
#ifdef WALDO_TWO_BRAIN
        brain_hp_print(now - last_stats);
#else
        float r0, r1;
        uint32_t errs, mfcc_us, npu_us, windows;
        mic_levels(&r0, &r1, &errs);
        kws_listen_stats(&mfcc_us, &npu_us, &windows);
        printf("voice: mic rms %.0f/%.0f (%lu errors), %lu windows, MFCC %lu us, NPU %lu us total\n",
               (double)r0, (double)r1, (unsigned long)errs, (unsigned long)windows,
               (unsigned long)mfcc_us, (unsigned long)npu_us);
#endif
        last_stats = now;
    }
    // Voice: the keyword spotter listens all the time (on the M55-HE in the two-brain build).
#ifdef WALDO_TWO_BRAIN
    int word = brain_hp_word();
#else
    int word = kws_listen(now);
#endif
    if (word >= 0) {
        printf("HEARD: %s\n", kws_label(word));
        if (orch_running) orch_word(kws_label(word), now);
        heard_word = kws_label(word);
        heard_ms = now;
    }

    if (orch_running) orch_tick(now);
#ifdef WALDO_TWO_BRAIN
    // Nothing until the next interrupt (camera bytes, the 1 ms tick): the HP sleeps.
    // The camera ring holds 64 KB, so a 1 ms nap loses nothing.
    __WFI();
#endif
}

// Mean absolute difference between two grayscale frames (every 4th pixel).
static uint32_t frame_diff(const uint8_t *a, const uint8_t *b, uint32_t n)
{
    uint32_t sum = 0, k = 0;
    for (uint32_t i = 0; i < n; i += 4, k++) sum += a[i] > b[i] ? a[i] - b[i] : b[i] - a[i];
    return k ? sum / k : 0;
}

// Waits n ms like wait_ms(), and measures the loop delay the tracker has to live with: the
// time from a servo command to the first camera frame that shows the turret moving.
static void wait_ms_probe(uint32_t n, const char *what)
{
    static uint8_t prev[CAM_MAX_W * CAM_MAX_H];
    const uint32_t t0 = ms;
    uint32_t seen = 0, base = 0, base_n = 0, frames = 0;
    bool have_prev = false, done = false;
    while (ms - t0 < n) {
        waldo_poll();
        const struct cam_frame *f = camlink_latest();
        if (!f || f->count == seen) continue;
        seen = f->count;
        const uint32_t npx = (uint32_t)f->width * f->height;
        if (have_prev && !done) {
            const uint32_t d = frame_diff(f->pix, prev, npx);
            frames++;
            // The first frames after the command still show the old view: their difference is
            // the scene's own noise. A frame clearly above it shows the move.
            if (frames <= 1) {
                base += d;
                base_n++;
            } else if (d > 6 && d > 3 * (base / (base_n ? base_n : 1)) + 2) {
                printf("WALDO: loop delay probe (%s): the camera showed the move %lu ms after the command "
                       "(frame diff %lu, scene noise %lu)\n", what, (unsigned long)(ms - t0), (unsigned long)d,
                       (unsigned long)(base / (base_n ? base_n : 1)));
                done = true;
            }
        }
        memcpy(prev, f->pix, npx);
        have_prev = true;
    }
    if (!done && frames) printf("WALDO: loop delay probe (%s): no clear move seen\n", what);
}

void waldo_servo_test(void)
{
    static const float steps[][2] = {{30, PARK_TILT}, {0, PARK_TILT}, {0, 0}, {0, PARK_TILT}};
    static const char *const what[] = {"pan 30", "pan back", "tilt up", "tilt down"};
    for (unsigned i = 0; i < sizeof steps / sizeof steps[0]; i++) {
        printf("WALDO: aim pan %.0f tilt %.0f\n", (double)steps[i][0], (double)steps[i][1]);
        motion_set(AXIS_PAN, steps[i][0]);
        motion_set(AXIS_TILT, steps[i][1]);
        wait_ms_probe(1500, what[i]);
    }
}

void waldo_home(void)
{
    motion_set_eased(true);   // ease from where the servos are to home, not a jolt
    motion_set(AXIS_PAN, 0);
    motion_set(AXIS_TILT, 0);
    orch_init(ms);   // start in SENTRY, looking ahead
    orch_running = true;
}
