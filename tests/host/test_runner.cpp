// Host test: a person running past the turret (the firmware's orchestrator + tracker +
// motion layer, unchanged). The scripted detector sees the person when they are inside
// the camera's 60 degree view of where the turret points, at 15 frames per second.
//   make -C tests/host test_runner && tests/host/test_runner
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "camlink_rx.h"
#include "detector.h"
#include "motion.h"
#include "orchestrator.h"
#include "tracker.h"

static det_box script[1];
static int script_n;
const char *detect_class_name(int cls) { return cls == 0 ? "person" : "?"; }
int detect(const cam_frame *, det_box *out, int max)
{
    int n = script_n < max ? script_n : max;
    memcpy(out, script, n * sizeof(det_box));
    return n;
}
static void pwm(int, float) {}

static cam_frame frame;
static uint32_t now;
constexpr int kW = 160, kH = 120;
constexpr float kDegPx = 60.0f / kW;
constexpr uint32_t kFrameMs = 66;   // ~15 fps, as measured on the board
constexpr uint32_t kAgeMs = 80;     // a frame shows the turret as it was this long ago

// Where the turret pointed kAgeMs ago, for rendering the (late) frame.
static float pan_hist[64];
static unsigned hist_i;

// One frame of a person at `angle` (turret degrees, +left); `hidden` = behind something.
// Returns the angle between the person and where the camera looks.
static float frame_of(float angle, bool hidden)
{
    for (uint32_t t = 0; t < kFrameMs; t += 2) {   // motion layer at 500 Hz
        now += 2;
        motion_tick(0.002f);
        orch_tick(now);
        pan_hist[hist_i++ % 64] = motion_pos(AXIS_PAN);
    }
    const float seen_pan = pan_hist[(hist_i - 1 - kAgeMs / 2) % 64];
    const float off = angle - seen_pan;   // +: person is left of the view centre
    script_n = 0;
    if (!hidden && fabsf(off) < 28) {
        const float cx = kW / 2.0f - off / kDegPx;
        script[0] = {(int)(cx - 10), 30, 20, 60, 0.9f, 0};
        script_n = 1;
    }
    det_box boxes[4];
    int n;
    frame.count++;
    orch_frame(&frame, now, boxes, 4, &n);
    return fabsf(angle - motion_pos(AXIS_PAN));
}

static void settle_on(float angle)
{
    tracker_reset_target();
    motion_set(AXIS_PAN, angle);
    motion_set(AXIS_TILT, 0);
    for (int i = 0; i < 40; i++) frame_of(angle, false);
}

#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main()
{
    int fails = 0;
    frame.width = kW;
    frame.height = kH;
    memset(frame.pix, 100, sizeof frame.pix);
    motion_init(pwm, -1, -1);
    orch_init(now);

    // 1. Runs across: -60 -> +60 degrees at 80 deg/s (about 4 m/s at 3 m).
    for (float speed : {40.0f, 80.0f, 120.0f}) {
        orch_init(now);
        settle_on(-60);
        int frames = 0, in_view = 0;
        float worst = 0, a = -60;
        while (a < 60) {
            a += speed * kFrameMs / 1000.0f;
            const float e = frame_of(a, false);
            frames++;
            in_view += e < 28;
            if (e > worst) worst = e;
        }
        for (int i = 0; i < 15; i++) frame_of(60, false);   // stops at +60
        const float end_err = frame_of(60, false);
        printf("runner at %3.0f deg/s: in view %d/%d frames, worst lag %.0f deg, %.1f deg off after stopping, %s\n",
               speed, in_view, frames, worst, end_err, orch_state_name(orch_state()));
        if (speed <= 80) CHECK(in_view == frames);
        CHECK(orch_state() == ORCH_TRACK && end_err < 5);
    }

    // 2. Behind a pillar for 0.6 s while running at 60 deg/s: picked up again right after.
    {
        orch_init(now);
        settle_on(-50);
        float a = -50;
        for (int i = 0; i < 8; i++) frame_of(a += 60 * kFrameMs / 1000.0f, false);
        for (int i = 0; i < 9; i++) frame_of(a += 60 * kFrameMs / 1000.0f, true);
        const float e = frame_of(a += 60 * kFrameMs / 1000.0f, false);
        printf("behind a pillar 0.6 s: %.0f deg off when it reappears (view is +-28)\n", e);
        CHECK(e < 28 && orch_state() == ORCH_TRACK);
    }

    // 3. Sprints out of view at 100 deg/s and stops 25 degrees further on, hidden for
    //    the last part: the search the way it went finds it.
    {
        orch_init(now);
        settle_on(-40);
        float a = -40;
        for (int i = 0; i < 4; i++) frame_of(a += 100 * kFrameMs / 1000.0f, false);
        for (int i = 0; i < 4; i++) frame_of(a += 100 * kFrameMs / 1000.0f, true);
        a += 25;
        int found = -1;
        for (int i = 0; i < 60 && found < 0; i++)
            if (frame_of(a, false) < 10 && orch_state() == ORCH_TRACK) found = i;
        printf("sprinted out of view: found again after %d frames (%.1f s)\n", found,
               found * kFrameMs / 1000.0);
        CHECK(found >= 0);
    }

    if (!fails) printf("runner: all checks passed\n");
    return fails ? 1 : 0;
}
