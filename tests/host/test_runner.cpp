// Host test: people moving around the turret (the firmware's orchestrator + tracker +
// motion layer, unchanged). A scripted detector sees each person who is inside the
// camera's view of where the turret pointed when the frame was taken.
//   make -C tests/host test_runner && tests/host/test_runner
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "camlink_rx.h"
#include "detector.h"
#include "motion.h"
#include "orchestrator.h"
#include "tracker.h"

// ---- scripted detector ----
static det_box script[8];
static int script_n;
const char *detect_class_name(int cls) { return cls == 0 ? "person" : cls == 56 ? "chair" : "?"; }
int detect(const cam_frame *, det_box *out, int max)
{
    int n = script_n < max ? script_n : max;
    memcpy(out, script, n * sizeof(det_box));
    return n;
}
static void pwm(int, float) {}

// ---- the simulated world ----
constexpr int kW = 160, kH = 120;                 // rotated camera frame
constexpr float kDegPx = 60.0f / kW;              // 60 degrees across the long side
constexpr float kHalfW = kW / 2 * kDegPx - 2;     // ~28 deg: box centre must be in view
constexpr float kHalfH = kH / 2 * kDegPx - 2;     // ~20 deg
struct person { float pan, tilt; bool hidden; int cls; };

static cam_frame frame;
static uint32_t now;
static struct {
    uint32_t frame_ms = 66;    // ~15 fps, as measured on the board
    uint32_t age_ms = 160;     // how old a frame is when the detector sees it (SIM_AGE_MS)
    float noise_px = 0;        // box centre jitter
    float miss = 0;            // chance the detector misses a visible person
    float h_px = 60;           // box height
} sim;

static uint32_t rng = 12345;
static float frand() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0f; }

static float pan_hist[512], tilt_hist[512];
static unsigned hist_i;

// One frame: the motion layer runs at 500 Hz until the frame arrives, then the detector
// reports every visible person as the camera saw them age_ms ago.
static void frame_of(const person *p, int np)
{
    for (uint32_t t = 0; t < sim.frame_ms; t += 2) {
        now += 2;
        motion_tick(0.002f);
        orch_tick(now);
        pan_hist[hist_i % 512] = motion_pos(AXIS_PAN);
        tilt_hist[hist_i % 512] = motion_pos(AXIS_TILT);
        hist_i++;
    }
    const unsigned back = sim.age_ms / 2 < hist_i ? sim.age_ms / 2 : hist_i - 1;
    const float seen_pan = pan_hist[(hist_i - 1 - back) % 512], seen_tilt = tilt_hist[(hist_i - 1 - back) % 512];
    script_n = 0;
    for (int i = 0; i < np && script_n < 8; i++) {
        const float op = p[i].pan - seen_pan, ot = p[i].tilt - seen_tilt;   // +: left of / above centre
        if (p[i].hidden || fabsf(op) > kHalfW || fabsf(ot) > kHalfH || frand() < sim.miss) continue;
        const float cx = kW / 2.0f - op / kDegPx + (frand() - 0.5f) * 2 * sim.noise_px;
        const float cy = kH / 2.0f - ot / kDegPx + (frand() - 0.5f) * 2 * sim.noise_px;
        const float h = sim.h_px, w = h / 3;
        // The tracker aims a person at 65% of the way down the box (objects at the centre).
        const float y = cy - (p[i].cls == 0 ? 0.65f : 0.5f) * h;
        script[script_n++] = {(int)lroundf(cx - w / 2), (int)lroundf(y), (int)w, (int)h, 0.9f, p[i].cls};
    }
    det_box boxes[8];
    int n;
    frame.count++;
    orch_frame(&frame, now, boxes, 8, &n);
}

static float err_pan(const person &p) { return fabsf(p.pan - motion_pos(AXIS_PAN)); }
static float err_tilt(const person &p) { return fabsf(p.tilt - motion_pos(AXIS_TILT)); }

// Start each scenario from a clean state, tracking a person standing at (pan, tilt).
static void start_on(person &p)
{
    sim = {};
    if (getenv("SIM_AGE_MS")) sim.age_ms = (uint32_t)atoi(getenv("SIM_AGE_MS"));
    orch_init(now);
    tracker_reset_target();
    motion_set(AXIS_PAN, p.pan);
    motion_set(AXIS_TILT, p.tilt);
    for (int i = 0; i < 40; i++) frame_of(&p, 1);
}

static int fails;
#define CHECK(c) do { if (!(c)) { printf("  FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main()
{
    frame.width = kW;
    frame.height = kH;
    memset(frame.pix, 100, sizeof frame.pix);
    motion_init(pwm, -1, -1);

    // 1. Running across, -60 -> +60, at walking to sprinting speeds.
    for (float speed : {40.0f, 80.0f, 120.0f}) {
        person p = {-60, 0, false, 0};
        start_on(p);
        int frames = 0, in_view = 0;
        float worst = 0;
        while (p.pan < 60) {
            p.pan += speed * sim.frame_ms / 1000.0f;
            frame_of(&p, 1);
            frames++;
            in_view += err_pan(p) < kHalfW;
            if (err_pan(p) > worst) worst = err_pan(p);
        }
        p.pan = 60;
        for (int i = 0; i < 15; i++) frame_of(&p, 1);
        printf("runner at %3.0f deg/s: in view %d/%d frames, worst lag %.0f deg, %.1f deg off after stopping\n",
               speed, in_view, frames, worst, err_pan(p));
        // 120 deg/s (a sprint at 3 m) is reported, not required: with ~160 ms old frames
        // and servos slewing at 120 deg/s, it outruns the turret.
        if (speed <= 80) CHECK(in_view == frames && orch_state() == ORCH_TRACK && err_pan(p) < 5);
    }

    // 2. Behind a pillar for 0.6 s while running at 60 deg/s.
    {
        person p = {-50, 0, false, 0};
        start_on(p);
        for (int i = 0; i < 8; i++) { p.pan += 60 * 0.066f; frame_of(&p, 1); }
        p.hidden = true;
        for (int i = 0; i < 9; i++) { p.pan += 60 * 0.066f; frame_of(&p, 1); }
        p.hidden = false;
        p.pan += 60 * 0.066f;
        frame_of(&p, 1);
        printf("behind a pillar 0.6 s: %.0f deg off when they reappear\n", err_pan(p));
        CHECK(err_pan(p) < kHalfW && orch_state() == ORCH_TRACK);
    }

    // 3. Sprints out of view at 100 deg/s, then stands 25 degrees further on.
    {
        person p = {-40, 0, false, 0};
        start_on(p);
        for (int i = 0; i < 4; i++) { p.pan += 100 * 0.066f; frame_of(&p, 1); }
        p.hidden = true;
        for (int i = 0; i < 4; i++) { p.pan += 100 * 0.066f; frame_of(&p, 1); }
        p.hidden = false;
        p.pan += 25;
        int found = -1;
        for (int i = 0; i < 60 && found < 0; i++) {
            frame_of(&p, 1);
            if (err_pan(p) < 10 && orch_state() == ORCH_TRACK) found = i;
        }
        printf("sprinted out of view: found again after %d frames\n", found);
        CHECK(found >= 0);
    }

    // 4. Standing still, noisy boxes (+-4 px) and 30% missed detections: the beam must
    //    not wander or buzz.
    {
        person p = {20, -10, false, 0};
        start_on(p);
        sim.noise_px = 4;
        sim.miss = 0.3f;
        float lo = 1e9f, hi = -1e9f, prev = motion_pos(AXIS_PAN), prev_d = 0;
        int reversals = 0;
        for (int i = 0; i < 150; i++) {   // 10 s
            frame_of(&p, 1);
            const float pos = motion_pos(AXIS_PAN), d = pos - prev;
            if (d * prev_d < 0) reversals++;
            if (d != 0) prev_d = d;
            prev = pos;
            if (pos < lo) lo = pos;
            if (pos > hi) hi = pos;
        }
        printf("standing still, noisy: pan wanders %.1f deg, %d direction reversals in 10 s, %s\n", hi - lo,
               reversals, orch_state_name(orch_state()));
        CHECK(hi - lo < 4 && reversals < 15 && orch_state() == ORCH_TRACK);
    }

    // 5. Walking (30 deg/s) with noisy boxes and 30% misses.
    {
        person p = {-40, 0, false, 0};
        start_on(p);
        sim.noise_px = 4;
        sim.miss = 0.3f;
        float worst = 0;
        while (p.pan < 40) {
            p.pan += 30 * 0.066f;
            frame_of(&p, 1);
            if (err_pan(p) > worst) worst = err_pan(p);
        }
        printf("walking, noisy, 30%% missed: worst lag %.0f deg, %s\n", worst, orch_state_name(orch_state()));
        CHECK(worst < 15 && orch_state() == ORCH_TRACK);
    }

    // 6. Two people cross: the one it follows walks right, the other walks left through them.
    {
        person p[2] = {{-30, 0, false, 0}, {30, 0, false, 0}};
        start_on(p[0]);
        for (int i = 0; i < 60; i++) {
            p[0].pan += 20 * 0.066f;
            p[1].pan -= 20 * 0.066f;
            frame_of(p, 2);
        }
        printf("two people cross: %.0f deg from the one it followed, %.0f from the other\n", err_pan(p[0]),
               err_pan(p[1]));
        CHECK(err_pan(p[0]) < 8);
    }

    // 7. A bystander stands still nearby while the one it follows walks away.
    {
        person a = {0, 0, false, 0};
        start_on(a);
        person p[2] = {a, {-15, 0, false, 0}};
        for (int i = 0; i < 45; i++) {
            p[0].pan += 25 * 0.066f;
            frame_of(p, 2);
        }
        printf("walks away from a bystander: %.0f deg from the one it followed\n", err_pan(p[0]));
        CHECK(err_pan(p[0]) < 8);
    }

    // 8. One-frame false detection 25 degrees away: no jump.
    {
        person p = {10, 0, false, 0};
        start_on(p);
        person two[2] = {p, {-15, 5, false, 0}};
        frame_of(two, 2);
        float worst = 0;
        for (int i = 0; i < 10; i++) {
            frame_of(&p, 1);
            if (err_pan(p) > worst) worst = err_pan(p);
        }
        printf("one-frame ghost 25 deg away: turret strays %.1f deg\n", worst);
        CHECK(worst < 6);
    }

    // 9. Runs one way, then turns back sharply.
    {
        person p = {-30, 0, false, 0};
        start_on(p);
        for (int i = 0; i < 15; i++) { p.pan += 60 * 0.066f; frame_of(&p, 1); }
        float worst = 0;
        for (int i = 0; i < 30; i++) {
            p.pan -= 60 * 0.066f;
            frame_of(&p, 1);
            if (err_pan(p) > worst) worst = err_pan(p);
        }
        printf("sharp turn at 60 deg/s: worst error %.0f deg, %.1f at the end\n", worst, err_pan(p));
        CHECK(worst < kHalfW && err_pan(p) < 10);
    }

    // 10. Sits down / stands up: tilt follows, within its limits (-90..+30).
    {
        person p = {0, 0, false, 0};
        start_on(p);
        float worst = 0;
        for (int i = 0; i < 30; i++) { p.tilt -= 30 * 0.066f; frame_of(&p, 1); worst = fmaxf(worst, err_tilt(p)); }
        for (int i = 0; i < 30; i++) { p.tilt += 30 * 0.066f; frame_of(&p, 1); worst = fmaxf(worst, err_tilt(p)); }
        printf("moving down and up at 30 deg/s: worst tilt error %.0f deg\n", worst);
        CHECK(worst < 10);
    }

    // 11. Runs past the pan limit (+90): the turret stops at the limit, then gives up and
    //     patrols from the nearest stop instead of driving home.
    {
        person p = {50, 0, false, 0};
        start_on(p);
        for (int i = 0; i < 40; i++) { p.pan += 60 * 0.066f; frame_of(&p, 1); }
        const float at_limit = motion_pos(AXIS_PAN);
        char first[24] = "home";
        for (int i = 0; i < 90 && orch_state() == ORCH_TRACK; i++) frame_of(&p, 1);   // up to 6 s
        if (orch_detail()) snprintf(first, sizeof first, "%s", orch_detail());
        printf("ran past the pan limit: turret stopped at %.0f, then %s from %s (pan %+.0f)\n", at_limit,
               orch_state_name(orch_state()), first, (double)motion_target(AXIS_PAN));
        // The nearest stop to +90 is stop 1 (+70): patrol starts there, not at home.
        CHECK(at_limit <= 90.01f && orch_state() == ORCH_SENTRY && !strcmp(first, "scan 1/10"));
    }

    // 12. Vanishes while standing still (walks behind a wall): no search sweep.
    {
        person p = {-20, 0, false, 0};
        start_on(p);
        p.hidden = true;
        float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < 45; i++) {   // 3 s, before the 4 s give-up
            frame_of(&p, 1);
            lo = fminf(lo, motion_pos(AXIS_PAN));
            hi = fmaxf(hi, motion_pos(AXIS_PAN));
        }
        printf("vanished while standing: turret moved %.1f deg in 3 s\n", hi - lo);
        CHECK(hi - lo < 3);
    }

    // 13. "stop" mid-chase freezes the turret; "go" picks the chase up again.
    {
        person p = {-40, 0, false, 0};
        start_on(p);
        for (int i = 0; i < 10; i++) { p.pan += 40 * 0.066f; frame_of(&p, 1); }
        orch_word("stop", now);
        const float frozen = motion_pos(AXIS_PAN);
        float moved = 0;
        for (int i = 0; i < 20; i++) {   // held 1.3 s; they slow down, still in view
            p.pan += 10 * 0.066f;
            frame_of(&p, 1);
            moved = fmaxf(moved, fabsf(motion_pos(AXIS_PAN) - frozen));
        }
        orch_word("go", now);
        for (int i = 0; i < 20; i++) frame_of(&p, 1);
        printf("\"stop\" mid-chase: moved %.1f deg while held; after \"go\", %.0f deg off (%s)\n", moved, err_pan(p),
               orch_state_name(orch_state()));
        CHECK(moved < 1.5f && orch_state() == ORCH_TRACK && err_pan(p) < 5);
    }

    // 14. Slow frames (5 fps) and frames older than the tracker assumes (200 ms vs 80).
    for (int mode = 0; mode < 2; mode++) {
        person p = {-50, 0, false, 0};
        start_on(p);
        if (mode == 0) sim.frame_ms = 200; else sim.age_ms = 200;
        float worst = 0;
        while (p.pan < 40) {
            p.pan += 40 * sim.frame_ms / 1000.0f;
            frame_of(&p, 1);
            worst = fmaxf(worst, err_pan(p));
        }
        for (int i = 0; i < 20; i++) frame_of(&p, 1);
        printf("%s, walking 40 deg/s: worst lag %.0f deg, %.1f deg off once stopped (%s)\n",
               mode == 0 ? "5 fps" : "200 ms old frames", worst, err_pan(p), orch_state_name(orch_state()));
        CHECK(worst < kHalfW && err_pan(p) < 5 && orch_state() == ORCH_TRACK);
    }

    // 15. Only a chair in view: never tracked (people are the default target).
    {
        sim = {};
        orch_init(now);
        tracker_reset_target();
        person c = {0, 0, false, 56};
        for (int i = 0; i < 60; i++) frame_of(&c, 1);
        printf("a chair in view: %s\n", orch_state_name(orch_state()));
        CHECK(orch_state() == ORCH_SENTRY);
    }

    // 16. The millisecond clock wraps around (after 49.7 days) in the middle of a chase.
    {
        now = 0xFFFFFFFFu - 3000;
        person p = {-40, 0, false, 0};
        start_on(p);
        float worst = 0;
        for (int i = 0; i < 40; i++) {
            p.pan += 40 * 0.066f;
            frame_of(&p, 1);
            worst = fmaxf(worst, err_pan(p));
        }
        printf("clock wraps mid-chase: worst lag %.0f deg (%s)\n", worst, orch_state_name(orch_state()));
        CHECK(worst < 15 && orch_state() == ORCH_TRACK);
    }

    printf(fails ? "runner: %d check(s) FAILED\n" : "runner: all checks passed\n", fails);
    return fails ? 1 : 0;
}
