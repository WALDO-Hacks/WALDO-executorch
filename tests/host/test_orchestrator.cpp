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
#include "tracker.h"

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

// A simulated room for the patrol: what the camera sees depends on where the turret
// points; a bright box appears at one spot when `object_at_stop` is set.
static bool room, object_at_stop;
static void render_room()
{
    const float pan = motion_pos(AXIS_PAN), tilt = motion_pos(AXIS_TILT);
    for (int y = 0; y < 120; y++)
        for (int x = 0; x < 160; x++)
            frame.pix[y * 160 + x] = (uint8_t)(60 + ((x + (int)(pan * 2.7f)) / 16 % 4) * 30 + ((y - (int)(tilt * 2.7f)) / 20 % 3) * 15);
    if (object_at_stop && fabsf(pan + 35) < 1 && fabsf(tilt) < 1)
        for (int y = 40; y < 90; y++) memset(frame.pix + y * 160 + 60, 250, 40);
}

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
        if (room) render_room();
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
    CHECK(orch_hold_target());   // the shipped default: a lost target is kept

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

    // "go" -> straight back to TRACK on the same target, without a trip home.
    orch_word("go", now);
    CHECK(orch_state() == ORCH_TRACK);
    CHECK(fabsf(motion_target(AXIS_PAN) - pan) < 0.01f);
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

    // The original behaviour (hold target off): a lost target is given up after 5 s.
    orch_set_hold_target(false);

    // Only objects in view (no person): not a target, so TRACK times out to SENTRY.
    script[0] = {60, 50, 16, 16, 0.9f, 15};   // a chair
    script_n = 1;
    step(60);
    CHECK(orch_state() == ORCH_SENTRY);

    // The camera stops sending while tracking: no frames, so orch_frame() never runs;
    // orch_tick() still gives up after the lost-target time.
    script[0] = {80, 50, 20, 20, 0.9f, 0};
    script_n = 1;
    step(8);
    CHECK(orch_state() == ORCH_TRACK);
    for (int i = 0; i < 350; i++) {   // 7 s of main-loop passes, no frames
        now += 20;
        orch_tick(now);
    }
    CHECK(orch_state() == ORCH_SENTRY);

    // ---- hold target (the default): what it follows is kept until another thing is specified ----
    orch_set_hold_target(true);
    script[0] = {110, 50, 20, 20, 0.9f, 0};   // a person right of centre
    script_n = 1;
    step(8);
    CHECK(orch_state() == ORCH_TRACK);
    // The person leaves for 30 s: still TRACK, pose held, no trip home.
    float hp = motion_target(AXIS_PAN), ht = motion_target(AXIS_TILT);
    script_n = 0;
    step(300);
    CHECK(orch_state() == ORCH_TRACK);
    CHECK(fabsf(motion_target(AXIS_PAN) - hp) < 0.01f && fabsf(motion_target(AXIS_TILT) - ht) < 0.01f);
    // The camera also stops sending for 20 s: orch_tick() does not give up either.
    for (int i = 0; i < 1000; i++) { now += 20; orch_tick(now); }
    CHECK(orch_state() == ORCH_TRACK);
    // The person comes back and is followed again.
    script[0] = {40, 50, 20, 20, 0.9f, 0};
    script_n = 1;
    float before_back = motion_target(AXIS_PAN);
    step(4);
    CHECK(motion_target(AXIS_PAN) > before_back + 2);

    // "track cup": the class is chosen by name. A person alone moving is not followed.
    orch_track_class(14, now);
    CHECK(tracker_target_class() == 14);
    script[0] = {130, 50, 16, 16, 0.9f, 0};   // person only, far right
    script_n = 1;
    float p_cup0 = motion_target(AXIS_PAN);
    step(10);
    CHECK(fabsf(motion_target(AXIS_PAN) - p_cup0) < 0.01f);
    // A cup left of centre: the turret turns to it (+pan = left).
    script[1] = {30, 50, 16, 16, 0.9f, 14};
    script_n = 2;
    step(6);
    CHECK(motion_target(AXIS_PAN) > p_cup0 + 2);
    // The cup is gone for 40 s (only the person is visible): it still waits for a cup.
    float p_wait = motion_target(AXIS_PAN);
    script_n = 1;
    step(400);
    CHECK(orch_state() == ORCH_TRACK && tracker_target_class() == 14);
    CHECK(fabsf(motion_target(AXIS_PAN) - p_wait) < 0.01f);
    // "stop" then "go": the class survives.
    orch_word("stop", now);
    orch_word("go", now);
    CHECK(orch_state() == ORCH_TRACK && tracker_target_class() == 14);
    // Two cups, the first followed; both visible after a gap: it stays on the first even though the
    // second is bigger.
    script[0] = {30, 50, 12, 12, 0.9f, 14};   // cup A, small, left
    script[1] = {120, 50, 24, 24, 0.9f, 14};  // cup B, bigger, right
    script_n = 2;
    orch_track_class(14, now);
    script_n = 1; step(3);                    // only A is visible: A is followed
    script_n = 0; step(50);                   // 5 s with nothing in view
    script_n = 2; float p_a = motion_target(AXIS_PAN);
    step(4);
    CHECK(motion_target(AXIS_PAN) > p_a - 0.01f);   // toward A (left), not toward B on the right
    // "no": back to people, class changes.
    orch_word("no", now);
    CHECK(tracker_target_class() == 0);
    // A cup chosen while PARKED is ignored; "off" then "on" starts again with people.
    CHECK(tracker_target_class() == 0);       // after "no"
    orch_word("off", now);
    orch_track_class(14, now);                // PARKED: ignored
    CHECK(tracker_target_class() == 0);
    orch_word("on", now);
    CHECK(orch_state() == ORCH_SENTRY);
    // In SENTRY a chosen class survives "stop" / "go" (nothing to resume, but the target is kept).
    orch_track_class(14, now);
    orch_word("stop", now);
    orch_word("go", now);
    CHECK(orch_state() == ORCH_SENTRY && tracker_target_class() == 14);
    orch_track_class(0, now);

    // ---- wake word: commands act only while listening; "stop" and "off" always work ----
    script_n = 0;
    CHECK(orch_listening(now));                 // not required (the default): always listening
    orch_set_wake_required(true);
    CHECK(!orch_listening(now));
    orch_word("stop", now);                     // safety word: no wake word needed
    CHECK(orch_state() == ORCH_HOLD);
    orch_word("go", now);                       // ignored: nobody said "Hey Waldo"
    CHECK(orch_state() == ORCH_HOLD);
    orch_wake(now);
    CHECK(orch_listening(now));
    orch_word("go", now);                       // now accepted
    CHECK(orch_state() == ORCH_SENTRY);
    now += 6000;                                // the window closes after 5 s
    CHECK(!orch_listening(now));
    float tilt_w = motion_target(AXIS_TILT);
    orch_word("up", now);
    CHECK(fabsf(motion_target(AXIS_TILT) - tilt_w) < 0.01f);    // ignored
    orch_wake(now);
    now += 4000;
    orch_word("up", now);                       // accepted, and renews the window
    CHECK(motion_target(AXIS_TILT) > tilt_w + 5);
    now += 4000;                                // 4 s after the command: still open
    CHECK(orch_listening(now));
    now += 2000;                                // 6 s after it: closed
    CHECK(!orch_listening(now));
    orch_word("off", now);                      // always works
    CHECK(orch_state() == ORCH_PARKED);
    orch_word("on", now);                       // ignored
    CHECK(orch_state() == ORCH_PARKED);
    orch_wake(now);
    orch_word("on", now);
    CHECK(orch_state() == ORCH_SENTRY);
    orch_set_wake_required(false);              // back to every word acts
    CHECK(orch_listening(now + 100000));
    orch_word("off", now);
    orch_word("on", now);
    CHECK(orch_state() == ORCH_SENTRY);

    // "stop" in SENTRY then "go": back to SENTRY (nothing to resume).
    script_n = 0;
    orch_word("stop", now);
    CHECK(orch_state() == ORCH_HOLD);
    orch_word("go", now);
    CHECK(orch_state() == ORCH_SENTRY);

    // Back in SENTRY: looking straight ahead.
    script_n = 0;
    orch_word("off", now);
    orch_word("on", now);
    CHECK(orch_state() == ORCH_SENTRY);
    CHECK(fabsf(motion_target(AXIS_PAN)) < 0.01f && fabsf(motion_target(AXIS_TILT)) < 0.01f);

    // Motion in SENTRY: something changes on the left of the picture -> the turret turns there.
    step(10);   // settle (still at home: the patrol starts after 4 quiet seconds)
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

    // Patrol: after 4 quiet seconds SENTRY walks a Z of 10 stops. YOLO runs at every stop on
    // the first lap, nowhere on a lap where nothing changed, and only where something
    // appeared on the next.
    script_n = 0;
    room = true;
    orch_word("off", now);
    orch_word("on", now);
    auto run_lap = [&](int *calls, float *pan_min, float *pan_max, float *tilt_max) {
        // until the patrol is back at stop 1 after stop 10
        bool seen_last = false;
        detect_calls = 0;
        *pan_min = 1e9f, *pan_max = -1e9f, *tilt_max = -1e9f;
        for (int i = 0; i < 2000; i++) {
            step(1);
            const char *d = orch_detail();
            if (d && !strcmp(d, "scan 10/10")) seen_last = true;
            if (d && seen_last && !strcmp(d, "scan 1/10")) break;
            *pan_min = fminf(*pan_min, motion_pos(AXIS_PAN));
            *pan_max = fmaxf(*pan_max, motion_pos(AXIS_PAN));
            *tilt_max = fmaxf(*tilt_max, motion_pos(AXIS_TILT));
        }
        *calls = detect_calls;
    };
    int calls;
    float pmin, pmax, tmax;
    step(45);   // the quiet spell at home, then on to stop 1
    CHECK(orch_detail() && !strcmp(orch_detail(), "scan 1/10"));
    run_lap(&calls, &pmin, &pmax, &tmax);
    printf("patrol lap 1: YOLO at %d stops, pan %.0f..%.0f, tilt up to %.0f\n", calls, pmin, pmax, tmax);
    CHECK(calls == 10);
    CHECK(pmin < -65 && pmax > 65 && tmax > 15);
    run_lap(&calls, &pmin, &pmax, &tmax);
    printf("patrol lap 2 (nothing changed): YOLO at %d stops\n", calls);
    CHECK(calls == 0);
    object_at_stop = true;
    run_lap(&calls, &pmin, &pmax, &tmax);
    printf("patrol lap 3 (an object appeared at one stop): YOLO at %d stops\n", calls);
    CHECK(calls == 1);
    CHECK(orch_state() == ORCH_SENTRY);

    // A person at a stop -> TRACK.
    script[0] = {70, 50, 20, 20, 0.9f, 0};
    script_n = 1;
    object_at_stop = false;   // the view changes back: YOLO runs there and finds the person
    for (int i = 0; i < 400 && orch_state() != ORCH_TRACK; i++) step(1);
    CHECK(orch_state() == ORCH_TRACK);

    printf("orchestrator: all checks passed\n");
    return 0;
}
