#include "tracker.h"

#include <math.h>
#include <stdio.h>

#include "motion.h"

static struct {
    float fov_long = 56;        // OV3660, 66.5-68 deg diagonal (vendor specs) -> 55-57 deg across the 4:3 long side
    float deadband = 2.0f;      // degrees: closer than this to a still target, don't move
    int sign_pan = 1, sign_tilt = 1;
} cfg;

// Two parts, so a wrong guess of the camera delay can't make the turret swing:
//  - position: each detection turns the turret a fraction (kKp) of the way to centring
//    the target, from the error measured in the image itself;
//  - speed: an alpha-beta filter on the target's angle in the room (where the turret
//    pointed when the frame was taken + the offset in it) estimates how fast it moves.
//    Between frames the turret keeps turning at that speed (feed-forward), so a runner
//    isn't left behind. When detections stop it carries on that way for a moment
//    (coast), then looks a bit further that way (search) before the orchestrator gives up.
constexpr float kKp = 0.35f;            // fraction of the image error corrected per frame
constexpr float kAlpha = 0.6f;          // filter: position correction per measurement
constexpr float kBeta = 0.05f;          // filter: speed correction per measurement
constexpr float kFeedForward = 0.5f;    // fraction of the estimated speed turned with (robust to a wrong delay)
constexpr float kMaxSpeed = 150;        // deg/s, a sprinting person a few metres away
constexpr uint32_t kFrameAgeMs = 160;   // capture + JPEG + 2 Mbaud UART + decode + YOLO, est. 150-200 ms
constexpr uint32_t kFreshMs = 1000;     // a measurement older than this restarts the filter
constexpr uint32_t kCoastMs = 700;      // after the last sighting: keep turning at its speed
constexpr uint32_t kSearchMs = 2500;    // then: look up to kSearchDeg further the way it went
constexpr float kSearchDeg = 30;
constexpr float kMovingDegS = 15;       // slower than this counts as standing still

static struct axis_filter {
    float th, v;   // target angle (deg) and speed (deg/s)
} kf[2];
static bool kf_on;
static uint32_t kf_t;   // time of the last measurement

// Recent turret positions (one per 10 ms, 640 ms deep), to know where it pointed when
// the frame was taken.
constexpr unsigned kHist = 64;
constexpr uint32_t kHistEveryMs = 10;
static struct { uint32_t t; float pan, tilt; } hist[kHist];
static unsigned hist_n;

static void turret_at(uint32_t t, float *pan, float *tilt)
{
    *pan = motion_pos(AXIS_PAN);
    *tilt = motion_pos(AXIS_TILT);
    for (unsigned k = 0; k < hist_n && k < kHist; k++) {   // newest first
        const auto &h = hist[(hist_n - 1 - k) % kHist];
        *pan = h.pan;
        *tilt = h.tilt;
        if ((int32_t)(h.t - t) <= 0) break;
    }
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static uint32_t last_tick;
static bool searched;

static int target_index = -1;
static int switch_dir;   // pending tracker_switch()
static uint32_t switch_ms;
constexpr uint32_t kSwitchMs = 1500;   // a switch that finds nothing in this long is dropped
static int target_cls = TRACK_PEOPLE;

static bool is_target_class(int cls)
{
    if (cls == DET_MOTION) return true;
    if (target_cls == TRACK_PEOPLE) return cls == 0;   // YOLO's person
    return cls == target_cls;
}

static struct {
    bool has_target, at_home = true;
    float cx, cy, w;
    uint32_t last_seen_ms, seen;
    float err_pan, err_tilt;
} st;

bool tracker_update(const struct det_box *b, int n, int img_w, int img_h, uint32_t now)
{
    // Stay on the target we were following if it's still there, else take the biggest
    // box of the target class.
    // "Still there" = near where the filter predicts it now, in turret angles, so a target
    // that moved (or a turret that turned) between frames is still recognised.
    const float deg_px = cfg.fov_long / (img_w > img_h ? img_w : img_h);
    float pan_then, tilt_then;
    turret_at(now - kFrameAgeMs, &pan_then, &tilt_then);
    const float pdt = (now - kf_t) / 1000.0f;
    const bool predict = kf_on && now - kf_t <= kFreshMs + kSearchMs;
    int pick = -1;
    float best = -1;
    for (int i = 0; i < n; i++) {
        if (!is_target_class(b[i].cls)) continue;
        float cx = b[i].x + b[i].w / 2.0f, cy = b[i].y + b[i].h / 2.0f;
        float score = (float)b[i].w * b[i].h;
        if (predict) {
            const float a_pan = pan_then - (cx - img_w / 2.0f) * deg_px * cfg.sign_pan;
            const float a_tilt = tilt_then - (cy - img_h / 2.0f) * deg_px * cfg.sign_tilt;
            const float gate = 8 + b[i].w * deg_px + fabsf(kf[AXIS_PAN].v) * pdt * 0.5f;
            if (hypotf(a_pan - (kf[AXIS_PAN].th + kf[AXIS_PAN].v * pdt),
                       a_tilt - (kf[AXIS_TILT].th + kf[AXIS_TILT].v * pdt)) < gate) score *= 4;
        } else if (st.has_target && hypotf(cx - st.cx, cy - st.cy) < 1.5f * st.w + 8) {
            score *= 4;
        }
        if (score > best) {
            best = score;
            pick = i;
        }
    }

    // A pending switch that found nothing to switch to in time is dropped, not kept for later.
    if (switch_dir && now - switch_ms > kSwitchMs) {
        printf("track: nothing to switch to\n");
        switch_dir = 0;
    }
    // "yes": lock onto whatever is nearest the middle of the picture (where the beam points),
    // of any class, and follow that kind of thing from now on. Works without a current target.
    if (switch_dir == TRACK_SWITCH_CENTER && n > 0) {
        int alt = -1;
        float alt_d = 0.3f * (img_w > img_h ? img_w : img_h);   // only something near the middle
        for (int i = 0; i < n; i++) {
            if (b[i].cls == DET_MOTION) continue;
            const float d = hypotf(b[i].x + b[i].w / 2.0f - img_w / 2.0f, b[i].y + b[i].h / 2.0f - img_h / 2.0f);
            if (d < alt_d) {
                alt_d = d;
                alt = i;
            }
        }
        if (alt >= 0) {
            pick = alt;
            target_cls = b[alt].cls;
            kf_on = false;
            printf("track: locked onto a %s\n", detect_class_name(target_cls));
            switch_dir = 0;
        }
    }
    // A requested switch: the nearest other box in that direction, of any class.
    if (switch_dir && switch_dir != TRACK_SWITCH_CENTER && st.has_target && n > 0) {
        int alt = -1;
        float alt_d = 1e9f;
        for (int i = 0; i < n; i++) {
            float dx = b[i].x + b[i].w / 2.0f - st.cx, dy = b[i].y + b[i].h / 2.0f - st.cy;
            float margin = st.w / 2;
            bool ok = switch_dir == TRACK_SWITCH_LEFT ? dx < -margin
                    : switch_dir == TRACK_SWITCH_RIGHT ? dx > margin
                    : switch_dir == TRACK_SWITCH_UP ? dy < -margin
                    : switch_dir == TRACK_SWITCH_DOWN ? dy > margin
                    : hypotf(dx, dy) > margin;
            float d = hypotf(dx, dy);
            if (ok && d < alt_d) {
                alt_d = d;
                alt = i;
            }
        }
        if (alt >= 0) {
            pick = alt;
            target_cls = b[alt].cls;
            kf_on = false;   // a different target: its speed starts from zero
            printf("track: switched to a %s\n", detect_class_name(target_cls));
        } else {
            printf("track: no other target that way\n");
        }
        switch_dir = 0;
    }
    if (pick < 0) {
        target_index = -1;
        st.has_target = false;
        return false;   // what to do about a lost target is the orchestrator's call
    }
    target_index = pick;
    st.cx = b[pick].x + b[pick].w / 2.0f;
    // A flashlight shouldn't shine in people's eyes: aim a person at 65% of the way down the
    // box (the chest in a head-and-shoulders view, the waist in a full-body one); objects
    // at their centre.
    st.cy = b[pick].y + b[pick].h * (b[pick].cls == 0 ? 0.65f : 0.5f);
    st.w = b[pick].w;
    st.has_target = true;
    st.at_home = false;
    st.last_seen_ms = now;
    st.seen++;

    // Angle of the target off the optical axis; +x right, +y down in the image. The frame
    // may be rotated (WALDO_CAM_ROTATE), so the field of view goes with the long side.
    float deg_per_px = cfg.fov_long / (img_w > img_h ? img_w : img_h);
    float ex = (st.cx - img_w / 2.0f) * deg_per_px;
    float ey = (st.cy - img_h / 2.0f) * deg_per_px;
    // +pan turns left and +tilt points up, so a target right of / below centre needs negative.
    st.err_pan = -ex * cfg.sign_pan;
    st.err_tilt = -ey * cfg.sign_tilt;

    float pan, tilt;
    turret_at(now - kFrameAgeMs, &pan, &tilt);
    const float meas[2] = {pan + st.err_pan, tilt + st.err_tilt};
    if (!kf_on || now - kf_t > kFreshMs) {
        for (int a = 0; a < 2; a++) kf[a] = {meas[a], 0};
        kf_on = true;
    } else {
        const float dt = (now - kf_t) / 1000.0f;
        for (int a = 0; a < 2 && dt > 0; a++) {
            const float pred = kf[a].th + kf[a].v * dt;
            const float r = meas[a] - pred;
            kf[a].th = pred + kAlpha * r;
            kf[a].v = clampf(kf[a].v + kBeta * r / dt, -kMaxSpeed, kMaxSpeed);
        }
    }
    kf_t = now;
    searched = false;
    if (fabsf(st.err_pan) > cfg.deadband) motion_nudge(AXIS_PAN, kKp * st.err_pan);
    if (fabsf(st.err_tilt) > cfg.deadband) motion_nudge(AXIS_TILT, kKp * st.err_tilt);
    return true;
}

void tracker_tick(uint32_t now, bool following)
{
    if (!hist_n || now - hist[(hist_n - 1) % kHist].t >= kHistEveryMs) {
        hist[hist_n % kHist] = {now, motion_pos(AXIS_PAN), motion_pos(AXIS_TILT)};
        hist_n++;
    }
    const float dt = (now - last_tick) / 1000.0f;
    last_tick = now;
    // The board ticks once per main loop, ~100 ms apart while a frame is processed: a
    // longer gap (a stall) is not turned into one big jump.
    if (!following || !kf_on || dt <= 0 || dt > 0.5f) return;
    const uint32_t since = now - kf_t;
    if (since < kCoastMs) {   // also bounds it if frames stop arriving altogether
        // Feed-forward: turn with a moving target between frames and through short misses
        // (motion blur, a pillar). A still target's "speed" is box noise: ignore it.
        // While it is seen, the image error closes the loop, so only part of the speed is
        // fed forward (a wrong delay guess can't build up into a swing); unseen, nothing
        // corrects the coast, so it uses the full speed.
        const float ff = st.has_target ? kFeedForward : 1.0f;
        for (int a = 0; a < 2; a++)
            if (fabsf(kf[a].v) >= kMovingDegS) motion_nudge(a, ff * kf[a].v * dt);
    } else if (!searched && since < kCoastMs + kSearchMs && fabsf(kf[AXIS_PAN].v) >= kMovingDegS) {
        // Out of sight while moving: it most likely carried on. Look further that way.
        searched = true;
        motion_nudge(AXIS_PAN, kf[AXIS_PAN].v > 0 ? kSearchDeg : -kSearchDeg);
    }
}

void tracker_print(void)
{
    printf("track: %s, seen %lu, error pan %+.1f tilt %+.1f, turret pan %.1f tilt %.1f, target speed %+.0f %+.0f deg/s\n",
           st.has_target ? "TARGET" : (st.at_home ? "home" : "lost"), (unsigned long)st.seen,
           (double)st.err_pan, (double)st.err_tilt, (double)motion_pos(AXIS_PAN),
           (double)motion_pos(AXIS_TILT), (double)(kf_on ? kf[0].v : 0), (double)(kf_on ? kf[1].v : 0));
}

int tracker_target_index(void) { return target_index; }

void tracker_switch(int dir, uint32_t now_ms)
{
    switch_dir = dir;
    switch_ms = now_ms;
}

void tracker_reset_target(void)
{
    target_cls = TRACK_PEOPLE;
    tracker_lose_target();
}

void tracker_lose_target(void)
{
    st.has_target = false;
    target_index = -1;
    switch_dir = 0;
    kf_on = false;
}

void tracker_set_target_class(int cls)
{
    target_cls = cls == 0 ? TRACK_PEOPLE : cls;
    tracker_lose_target();
}

int tracker_target_class(void) { return target_cls == TRACK_PEOPLE ? 0 : target_cls; }
