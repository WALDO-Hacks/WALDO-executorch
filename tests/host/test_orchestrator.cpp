// Host test of WALDO's orchestrator + tracker + motion layer (the firmware sources,
// unchanged), with a scripted detector, synthetic frames and voice words.
//   make -C tests/host
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "camlink_rx.h"
#include "detector.h"
#include "motion.h"
#include "orchestrator.h"

// ---- scripted detector ----
static det_box script[4];
static int script_n, detect_calls;
const char *detect_class_name(int cls) { return cls == 14 ? "cup" : cls == 0 ? "person" : "?"; }
int detect(const cam_frame *, det_box *out, int max)
{
    detect_calls++;
    int n = script_n < max ? script_n : max;
    memcpy(out, script, n * sizeof(det_box));
    return n;
}

static void pwm(int, float) {}
static cam_frame frame;
static uint32_t now;

// One camera frame every 100 ms; the motion layer ticks every 20 ms in between.
static int step(int frames = 1)
{
    det_box boxes[8];
    int n = -1;
    for (int i = 0; i < frames; i++) {
        for (int t = 0; t < 5; t++) {
            now += 20;
            motion_tick(0.02f);
        }
        frame.count++;
        orch_frame(&frame, now, boxes, 8, &n);
    }
    return n;
}

#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); return 1; } } while (0)

int main()
{
    frame.width = 160;
    frame.height = 120;
    memset(frame.pix, 100, sizeof frame.pix);
    motion_init(pwm, -1, -1);
    motion_set(AXIS_PAN, 0);
    motion_set(AXIS_TILT, 0);
    step(30);
    orch_init(now);

    // SENTRY: nothing there -> the detector runs on every 4th frame only.
    detect_calls = 0;
    step(20);
    CHECK(orch_state() == ORCH_SENTRY);
    CHECK(detect_calls == 5);

    // A face right of centre -> TRACK, and the turret pans right (negative pan).
    script[0] = {110, 50, 20, 20, 0.9f, 0};
    script_n = 1;
    step(8);
    CHECK(orch_state() == ORCH_TRACK);
    CHECK(motion_target(AXIS_PAN) < -3);

    // "stop" -> HOLD: keeps detecting, doesn't move.
    orch_word("stop", now);
    CHECK(orch_state() == ORCH_HOLD);
    float pan = motion_target(AXIS_PAN);
    detect_calls = 0;
    step(10);
    CHECK(detect_calls == 10);
    CHECK(fabsf(motion_target(AXIS_PAN) - pan) < 0.01f);

    // "go" -> back to SENTRY, then TRACK again.
    orch_word("go", now);
    step(8);
    CHECK(orch_state() == ORCH_TRACK);

    // Two faces: "left" switches to the left one and the turret turns toward it.
    script[0] = {85, 50, 16, 16, 0.9f, 0};    // the person being followed, near centre
    script[1] = {20, 50, 16, 16, 0.9f, 14};   // a cup to its left
    script_n = 2;
    step(4);
    float before = motion_target(AXIS_PAN);
    orch_word("left", now);
    step(1);
    CHECK(motion_target(AXIS_PAN) > before + 2);   // +pan turns left

    // It now follows the cup, even though a person is in view: the person alone moving
    // right doesn't pull it back.
    script[0] = {130, 50, 16, 16, 0.9f, 0};
    script[1] = {60, 50, 16, 16, 0.9f, 14};   // the cup, now left of centre
    float pan_cup = motion_target(AXIS_PAN);
    step(3);
    CHECK(motion_target(AXIS_PAN) > pan_cup);  // still turning left, toward the cup

    // "no" -> back to people: it turns right toward the person.
    orch_word("no", now);
    pan_cup = motion_target(AXIS_PAN);
    step(3);
    CHECK(motion_target(AXIS_PAN) < pan_cup);

    // Only objects in view (no person): not a target, so TRACK times out to SENTRY.
    script[0] = {60, 50, 16, 16, 0.9f, 15};   // a chair
    script_n = 1;
    step(60);
    CHECK(orch_state() == ORCH_SENTRY);

    // Target gone for 5 s -> SENTRY, looking straight ahead.
    script_n = 0;
    step(60);
    CHECK(orch_state() == ORCH_SENTRY);
    CHECK(fabsf(motion_target(AXIS_PAN)) < 0.01f && fabsf(motion_target(AXIS_TILT)) < 0.01f);

    // Motion in SENTRY: something changes on the left of the picture -> the turret turns there.
    step(10);   // settle
    for (int y = 40; y < 80; y++) memset(frame.pix + y * 160 + 10, 250, 30);
    step(1);
    CHECK(motion_target(AXIS_PAN) > 2);

    // "off" -> PARKED: head down, the detector never runs; "up" is ignored; "on" wakes it.
    orch_word("off", now);
    CHECK(orch_state() == ORCH_PARKED);
    CHECK(motion_target(AXIS_TILT) < -45);
    detect_calls = 0;
    script[0] = {70, 50, 20, 20, 0.9f, 0};
    script_n = 1;
    step(10);
    orch_word("up", now);
    CHECK(detect_calls == 0 && orch_state() == ORCH_PARKED && motion_target(AXIS_TILT) < -45);
    orch_word("on", now);
    CHECK(orch_state() == ORCH_SENTRY);

    // Nudges when not tracking: "up" in SENTRY tilts up 10 degrees.
    script_n = 0;
    float tilt = motion_target(AXIS_TILT);
    orch_word("up", now);
    CHECK(motion_target(AXIS_TILT) > tilt + 5);

    printf("orchestrator: all checks passed\n");
    return 0;
}
