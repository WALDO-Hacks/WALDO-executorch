#include "tracker.h"

#include <math.h>
#include <stdio.h>

#include "motion.h"

static struct {
    float fov_long = 60;        // ESP32 OV3660: degrees across the sensor's long side
    float deadband = 1.5f;      // degrees: closer than this to a still target, don't move
    int sign_pan = 1, sign_tilt = 1;
} cfg;

// The target is followed in turret angles (where it is in the room), not image pixels:
// angle = where the turret pointed when the frame was taken + the target's offset in it.
// An alpha-beta filter per axis smooths that and estimates the target's angular speed,
// and the turret aims where the target will be (lead), not where it was. When detections
// stop, the turret keeps going the way the target went (coast), then looks a bit further
// that way (search) before the orchestrator gives up.
constexpr float kAlpha = 0.75f;        // position correction per measurement
constexpr float kBeta = 0.35f;          // speed correction per measurement
constexpr float kMaxSpeed = 150;        // deg/s, a sprinting person a few metres away
constexpr float kLeadS = 0.12f;         // aim this far ahead: frame age + servo response
constexpr uint32_t kFrameAgeMs = 80;    // capture -> JPEG -> UART -> decode, before the detector
constexpr uint32_t kFreshMs = 1000;     // a measurement older than this restarts the filter
constexpr uint32_t kCoastMs = 700;      // after the last sighting: follow the predicted path
constexpr uint32_t kSearchMs = 2500;    // then: look up to kSearchDeg further the way it went
constexpr float kSearchDeg = 30;
constexpr float kMovingDegS = 15;       // slower than this counts as standing still

static struct axis_filter {
    float th, v;   // target angle (deg) and speed (deg/s)
} kf[2];
static bool kf_on;
static uint32_t kf_t;   // time of the last measurement

// Recent turret positions, to know where it pointed when the frame was taken.
static struct { uint32_t t; float pan, tilt; } hist[32];
static unsigned hist_n;

static void turret_at(uint32_t t, float *pan, float *tilt)
{
    *pan = motion_pos(AXIS_PAN);
    *tilt = motion_pos(AXIS_TILT);
    for (unsigned k = 0; k < hist_n && k < 32; k++) {   // newest first
        const auto &h = hist[(hist_n - 1 - k) % 32];
        *pan = h.pan;
        *tilt = h.tilt;
        if ((int32_t)(h.t - t) <= 0) break;
    }
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// Point the turret at the filter's prediction for `now` (+ lead).
static void aim(uint32_t now, float extra_pan_deg)
{
    const float dt = (now - kf_t) / 1000.0f + kLeadS;
    for (int a = 0; a < 2; a++) {
        float goal = kf[a].th + kf[a].v * dt + (a == AXIS_PAN ? extra_pan_deg : 0);
        // A still target close to centre: hold, so the beam doesn't jitter.
        if (fabsf(kf[a].v) < kMovingDegS && fabsf(goal - motion_target(a)) < cfg.deadband) continue;
        motion_set(a, goal);
    }
}

static int target_index = -1;
static int switch_dir;   // pending tracker_switch()
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

    // A requested switch: the nearest other box in that direction, of any class.
    if (switch_dir && st.has_target && n > 0) {
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
    aim(now, 0);
    return true;
}

void tracker_tick(uint32_t now, bool following)
{
    hist[hist_n % 32] = {now, motion_pos(AXIS_PAN), motion_pos(AXIS_TILT)};
    hist_n++;
    if (!following || !kf_on) return;
    const uint32_t since = now - kf_t;
    if (st.has_target || since < kCoastMs) {
        aim(now, 0);   // between frames, and through short misses (motion blur, a pillar)
    } else if (since < kCoastMs + kSearchMs && fabsf(kf[AXIS_PAN].v) >= kMovingDegS) {
        // Out of sight while moving: it most likely carried on. Hold the coast point's
        // speed off, and look further that way.
        const float coast = kCoastMs / 1000.0f + kLeadS;
        const float dir = kf[AXIS_PAN].v > 0 ? 1.0f : -1.0f;
        motion_set(AXIS_PAN, kf[AXIS_PAN].th + kf[AXIS_PAN].v * coast + dir * kSearchDeg);
        motion_set(AXIS_TILT, kf[AXIS_TILT].th);
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

void tracker_switch(int dir) { switch_dir = dir; }

void tracker_reset_target(void)
{
    target_cls = TRACK_PEOPLE;
    st.has_target = false;
    target_index = -1;
    kf_on = false;
}

int tracker_target_class(void) { return target_cls == TRACK_PEOPLE ? 0 : target_cls; }
