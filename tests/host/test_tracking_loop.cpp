// From the team's integration branch (two-brain-waldo-integration), with tracker_tick() added.
// Closed-loop host test of the steering chain: simulated camera -> tracker.cpp -> motion.cpp (limits + slew) -> turret pose -> camera.
// The firmware sources are used unchanged. HOST simulation (no real optics, servos or timing jitter); it checks the signs, the gain
// and the stability of the loop, not the turret. World target: azimuth a (+left), elevation e (+up), degrees; +pan = left, +tilt = up.
//   make -C tests/host
#include <cmath>
#include <cstdio>
#include <cstring>

#include "detector.h"
#include "motion.h"
#include "tracker.h"

const char *detect_class_name(int) { return "x"; }
static void pwm(int, float) {}
constexpr float kAssumedFov = 56;   // tracker.cpp's fov_long
// A settled turret is within the tracker's 2 deg deadband (in its assumed degrees, so
// 2 * true / assumed real degrees) plus about one frame of latency.
static float settle_bound(float fov_true) { return 2.0f * fov_true / kAssumedFov + 1.5f; }
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)
static int fails;

struct Scn { float fov_true; float frame_ms; float latency_ms; float a0, e0, a_rate; float hide_from, hide_to; float t_end; };
struct Res { float err_pan, err_tilt, worst_pan_overshoot; float rms_err; float seen_frac; };

static Res run(const Scn &c)
{
    const int W = 160, H = 120;
    motion_init(pwm, -1, -1);
    motion_set(AXIS_PAN, 0); motion_set(AXIS_TILT, 0);
    for (int i = 0; i < 100; i++) motion_tick(0.02f);
    tracker_reset_target();
    const float dpp = c.fov_true / W;   // degrees per pixel along the long side
    const int lag = (int)(c.latency_ms / 20.f);
    float hp[64] = {0}, ht[64] = {0};
    Res r = {0, 0, 0, 0, 0};
    int frames = 0, seen = 0, nsq = 0; float sq = 0;
    uint32_t now = 1000;
    for (int k = 0; k < (int)(c.t_end / 0.02f); k++) {
        const float t = k * 0.02f, a = c.a0 + c.a_rate * t;
        for (int i = 63; i > 0; i--) { hp[i] = hp[i - 1]; ht[i] = ht[i - 1]; }
        hp[0] = motion_pos(AXIS_PAN); ht[0] = motion_pos(AXIS_TILT);
        now += 20;
        if (((k * 20) % (int)c.frame_ms) == 0) {
            const float ex = hp[lag] - a, ey = ht[lag] - c.e0;   // + right, + below
            const float cx = W / 2.f + ex / dpp, cy = H / 2.f + ey / dpp;
            const bool hidden = t >= c.hide_from && t < c.hide_to;
            det_box b = {(int)lroundf(cx) - 8, (int)lroundf(cy) - 8, 16, 16, 0.9f, 0};
            const bool vis = !hidden && cx > 0 && cx < W && cy > 0 && cy < H;
            tracker_update(&b, vis ? 1 : 0, W, H, now);
            frames++; seen += vis;
        }
        tracker_tick(now, true);   // the main loop's tick: feed-forward and position history
        motion_tick(0.02f);
        const float ep = a - motion_pos(AXIS_PAN), et = c.e0 - motion_pos(AXIS_TILT);
        if (c.a_rate == 0 && c.a0 > 0 && -ep > r.worst_pan_overshoot) r.worst_pan_overshoot = -ep;   // beyond the target
        if (c.a_rate == 0 && c.a0 < 0 && ep > r.worst_pan_overshoot) r.worst_pan_overshoot = ep;
        if (t > c.t_end / 2) { sq += ep * ep + et * et; nsq++; }
        r.err_pan = ep; r.err_tilt = et;
    }
    r.rms_err = nsq ? sqrtf(sq / nsq) : 0;
    r.seen_frac = frames ? (float)seen / frames : 0;
    return r;
}

int main()
{
    // 1. signs: a person 15 deg left and 8 deg up: the turret turns left (+pan) and up (+tilt) and ends on it.
    { Scn c = {60, 200, 100, 15, 8, 0, 0, 0, 5}; Res r = run(c);
      CHECK(motion_target(AXIS_PAN) > 10 && motion_target(AXIS_TILT) > 4);
      CHECK(fabsf(r.err_pan) < settle_bound(60) && fabsf(r.err_tilt) < settle_bound(60)); }
    // ... and to the right and down: the other way.
    { Scn c = {60, 200, 100, -15, -8, 0, 0, 0, 5}; Res r = run(c);
      CHECK(motion_target(AXIS_PAN) < -10 && motion_target(AXIS_TILT) < -4);
      CHECK(fabsf(r.err_pan) < settle_bound(60) && fabsf(r.err_tilt) < settle_bound(60)); }

    // 2. the field of view of the real lens is a guess (the tracker assumes 56 deg): the loop settles for 40..100 deg,
    //    at 5 fps (the old gray link) and 10 fps, with 100 ms of image latency, and does not overshoot much.
    printf("true FOV  frame   settle error (pan/tilt)   pan overshoot\n");
    const float fovs[4] = {40, 60, 80, 100}, rates[2] = {200, 100};
    for (int i = 0; i < 4; i++) for (int j = 0; j < 2; j++) {
        Scn c = {fovs[i], rates[j], 100, 15, 7, 0, 0, 0, 6}; Res r = run(c);
        printf("  %4.0f   %4.0f ms   %5.2f / %5.2f           %5.2f deg\n", fovs[i], rates[j], r.err_pan, r.err_tilt, r.worst_pan_overshoot);
        // Bounds: the tracker's 2 deg deadband is in assumed degrees, so on a lens of true FOV F it is 2 * F / 60 deg; one frame
        // of latency adds about 1.5 deg. Overshoot < 5 deg. These are the RECORDED behaviour of the loop (measured 0.0 - 4.4 deg
        // overshoot, 0.0 - 3.7 deg final error), not a design target: the real FOV is unmeasured.
        const float bound = settle_bound(fovs[i]);
        CHECK(fabsf(r.err_pan) < bound && fabsf(r.err_tilt) < bound);
        CHECK(r.worst_pan_overshoot < 5.0f);
    }

    // 3. a person walking at 10 deg/s (right to left across 15 deg right .. 45 deg left): stays in view and lags by a bounded amount.
    for (int i = 0; i < 3; i++) { Scn c = {fovs[i], 100, 100, -15, 0, 10, 0, 0, 6}; Res r = run(c);
        printf("walking 10 deg/s, FOV %3.0f: rms error %.2f deg, in view %.0f %% of frames\n", fovs[i], r.rms_err, 100 * r.seen_frac);
        CHECK(r.rms_err < 8.0f && r.seen_frac > 0.95f); }

    // 4. hidden for 8 s while the person stands still, then visible at the same place: the pose is held and the loop resumes.
    { Scn c = {60, 100, 100, 15, 0, 0, 3, 11, 16}; Res r = run(c);
      CHECK(fabsf(r.err_pan) < settle_bound(60) && fabsf(r.err_tilt) < settle_bound(60)); }

    if (!fails) printf("tracking loop: all checks passed\n");
    return fails ? 1 : 0;
}
