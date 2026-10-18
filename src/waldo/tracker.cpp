#include "tracker.h"

#include <math.h>
#include <stdio.h>

#include "motion.h"

static struct {
    float fov_long = 50;        // OV3660 lenses: 66.5-68 deg diagonal -> 55-57 deg across the 4:3 long side;
                                // assume a bit less: a real view WIDER than assumed is the safe side
    float deadband = 2.0f;      // degrees: closer than this to a still target, don't move
    int sign_pan = 1, sign_tilt = 1;
} cfg;

// Steering, in turret angles. A camera frame is ~kFrameAgeMs old when the detector sees it,
// so the turret has moved since. The target's angle in the room is where the turret pointed
// when the frame was taken (a short position history) plus the target's offset in the frame.
// Each frame moves the aim kGain of the way to that angle. Aiming at where the target IS,
// rather than nudging by the error, doesn't re-correct an error the turret already corrected
// in the frames still on their way, so it doesn't swing on a long delay (tests/host/test_runner,
// real delays 150-350 ms).
// An alpha-beta filter on those angles estimates the target's speed. It is NOT fed to the
// servos while the target is seen; it only carries the turret through a short loss (coast,
// capped at kCoastMaxDeg) and, if the target left at an edge of the picture, makes it look a
// bit further that way (search). It also helps to recognise the same target next frame.
constexpr float kGain = 0.5f;           // fraction of the way to the target's angle per frame
constexpr float kAlpha = 0.6f;          // filter: position correction per measurement
constexpr float kBeta = 0.3f;           // filter: speed correction per measurement
constexpr float kMaxSpeed = 150;        // deg/s, a sprinting person a few metres away
constexpr uint32_t kFrameAgeMs = 250;   // capture + JPEG + 2 Mbaud UART + decode + YOLO; the boot
                                        // "loop delay probe" measures it on the board
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

// A clock that went backwards (a restart; the host tests) makes the history meaningless.
static void hist_check(uint32_t now)
{
    if (hist_n && (int32_t)(now - hist[(hist_n - 1) % kHist].t) < 0) hist_n = 0;
}

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
static float coast_deg;                 // turned so far while coasting without a sighting
constexpr float kCoastMaxDeg = 25;      // a coast never turns further than this

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

// Latch: once a person is picked, only that person is followed. Another person who appears, is
// bigger or walks in front is ignored. The latch ends when the target has not been matched for
// kLatchMs of processed frames (it disappeared; a "stop" pause does not count), or on a voice
// command (switch, "no", "yes", a new class). Without it the biggest person wins every frame.
// There is no appearance model: "the same person" means "inside the prediction gate".
constexpr uint32_t kLatchMs = 4000;     // unmatched this long -> the latch ends
constexpr float kLatchGateBaseDeg = 14; // gate around the predicted place, plus the target's box width
constexpr float kLatchGateDriftDegS = 2;    // plus this much per second unseen (a still person: little; a moving one widens it by its speed)
constexpr float kLatchMaxSpeedDegS = 80; // the speed estimate widens the gate by at most this per second
constexpr float kLatchGateMaxDeg = 35;  // never wider than this: about the field of view
// A detector box must look like a person before it can be followed: this keeps clutter, ghost
// blobs and the part-boxes of one person (torso, legs) from being picked or taking turns.
constexpr float kMinPersonScore = 0.5f;   // the detector's own floor is 0.35 (face_detect_adapter.cpp)
constexpr float kMaxAspectWH = 2.0f;      // a person is not much wider than tall
constexpr float kMinAreaFrac = 0.004f;    // of the image
constexpr float kNestedFrac = 0.8f;       // a box this much inside a bigger one is part of the same person
static bool latched;
static uint32_t last_update_ms, miss_ms;

static struct {
    bool has_target, at_home = true;
    float cx, cy, w;
    uint32_t last_seen_ms, seen;
    float err_pan, err_tilt;
    int exit_dir;   // +1/-1: last seen near the frame's left/right edge (turn +pan/-pan to follow), else 0
} st;

bool tracker_update(const struct det_box *b, int n, int img_w, int img_h, uint32_t now)
{
    // Stay on the target we were following if it's still there, else take the biggest
    // box of the target class.
    // "Still there" = near where the filter predicts it now, in turret angles, so a target
    // that moved (or a turret that turned) between frames is still recognised.
    hist_check(now);
    const float deg_px = cfg.fov_long / (img_w > img_h ? img_w : img_h);
    float pan_then, tilt_then;
    turret_at(now - kFrameAgeMs, &pan_then, &tilt_then);
    const float pdt = (now - kf_t) / 1000.0f;
    const bool predict = kf_on && now - kf_t <= kFreshMs + kSearchMs;
    // Time since the last update, capped so a pause with no updates ("stop") isn't counted as lost.
    const uint32_t since_update = last_update_ms && (int32_t)(now - last_update_ms) > 0
                                      ? (now - last_update_ms < 500 ? now - last_update_ms : 500) : 0;
    last_update_ms = now;
    // Which boxes may be followed at all (people-quality, not a fragment of a bigger box).
    bool ok[16];
    const int nn = n < 16 ? n : 16;
    for (int i = 0; i < nn; i++) {
        ok[i] = is_target_class(b[i].cls) &&
                (b[i].cls == DET_MOTION || (b[i].score >= kMinPersonScore && b[i].w > 0 && b[i].h > 0 &&
                                            b[i].w <= kMaxAspectWH * b[i].h &&
                                            (float)b[i].w * b[i].h >= kMinAreaFrac * img_w * img_h));
    }
    for (int i = 0; i < nn; i++) {
        if (!ok[i] || b[i].cls == DET_MOTION) continue;
        for (int j = 0; j < nn; j++) {
            if (j == i || !ok[j] || b[j].cls == DET_MOTION || b[j].w * b[j].h <= b[i].w * b[i].h) continue;
            const int ix = fmaxf(0, fminf(b[i].x + b[i].w, b[j].x + b[j].w) - fmaxf(b[i].x, b[j].x));
            const int iy = fmaxf(0, fminf(b[i].y + b[i].h, b[j].y + b[j].h) - fmaxf(b[i].y, b[j].y));
            if (ix * iy >= kNestedFrac * b[i].w * b[i].h) { ok[i] = false; break; }
        }
    }
    int pick = -1;
    float best = -1;
    for (int i = 0; i < n; i++) {
        if (i >= nn || !ok[i]) continue;
        float cx = b[i].x + b[i].w / 2.0f, cy = b[i].y + b[i].h / 2.0f;
        float score = (float)b[i].w * b[i].h;
        if (latched) {
            // Only the latched person may be picked: a box must be inside a gate around where it was
            // last seen, however big it is. A movement box never takes a latched target.
            if (b[i].cls == DET_MOTION) continue;
            float d, gate;
            if (predict) {
                const float a_pan = pan_then - (cx - img_w / 2.0f) * deg_px * cfg.sign_pan;
                const float a_tilt = tilt_then - (cy - img_h / 2.0f) * deg_px * cfg.sign_tilt;
                // Around where it was last seen, not where the speed says it is now: the speed
                // estimate is noisy (it can read 100+ deg/s for a person walking), so it only widens
                // the gate, capped.
                d = hypotf(a_pan - kf[AXIS_PAN].th, a_tilt - kf[AXIS_TILT].th);
                const float v = fminf(hypotf(kf[AXIS_PAN].v, kf[AXIS_TILT].v), kLatchMaxSpeedDegS);
                gate = kLatchGateBaseDeg + st.w * deg_px + (v + kLatchGateDriftDegS) * pdt;
            } else {   // no usable speed estimate (just switched, or paused): compare with where it was seen
                d = hypotf(cx - st.cx, cy - st.cy) * deg_px;
                gate = (1.5f * st.w + 8) * deg_px + kLatchGateDriftDegS * (miss_ms / 1000.0f);
            }
            if (gate > kLatchGateMaxDeg) gate = kLatchGateMaxDeg;
            if (d >= gate) continue;   // not the latched person; among those that are, the usual scoring below
        }
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
            if (b[i].cls == DET_MOTION || i >= nn || !ok[i]) continue;
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
            if (i >= nn || !ok[i]) continue;
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
        if (latched) {
            miss_ms += since_update;
            if (miss_ms > kLatchMs) {   // gone: whoever shows up next is a new target
                printf("track: latched target gone for %lu ms, latch released\n", (unsigned long)miss_ms);
                latched = false;
                kf_on = false;
            }
        }
        return false;   // what to do about a lost target is the orchestrator's call
    }
    // Latch on what was picked (not on a movement box: that only turns the gaze).
    if (b[pick].cls != DET_MOTION) {
        if (!latched) printf("track: latched onto a %s\n", detect_class_name(b[pick].cls));
        latched = true;
    }
    miss_ms = 0;
    target_index = pick;
    st.cx = b[pick].x + b[pick].w / 2.0f;
    // Aim at the centre of the box (people and objects alike), so the beam and the screen's
    // centre mark sit on the target. A close person's box is cut off by the frame edge; its
    // centre then moves with the tilt, which only softens the correction (no feedback loop).
    st.cy = b[pick].y + b[pick].h * 0.5f;
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
    coast_deg = 0;
    // Near the left or right edge of the picture: if it disappears now, it left that way.
    const float half_w = img_w / 2.0f * deg_per_px;
    st.exit_dir = fabsf(st.err_pan) > 0.5f * half_w ? (st.err_pan > 0 ? 1 : -1) : 0;
    // Aim part of the way to where the target is (meas: turret then + offset in the frame).
    for (int a = 0; a < 2; a++) {
        const float to = meas[a] - motion_target(a);
        if (fabsf(to) > cfg.deadband) motion_set(a, motion_target(a) + kGain * to);
    }
    return true;
}

void tracker_tick(uint32_t now, bool following)
{
    hist_check(now);
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
    if (!st.has_target && since < kCoastMs) {   // also bounds it if frames stop arriving altogether
        // Coast: a short miss (motion blur, a pillar): keep turning at the target's speed.
        // Capped (kCoastMaxDeg), so a bad speed estimate can't fling the turret across the room.
        // A still target's "speed" is box noise: ignore it.
        for (int a = 0; a < 2; a++) {
            if (fabsf(kf[a].v) < kMovingDegS || coast_deg >= kCoastMaxDeg) continue;
            const float step = kf[a].v * dt;
            coast_deg += fabsf(step);
            motion_nudge(a, step);
        }
    } else if (!searched && since < kCoastMs + kSearchMs && fabsf(kf[AXIS_PAN].v) >= kMovingDegS &&
               st.exit_dir == (kf[AXIS_PAN].v > 0 ? 1 : -1)) {
        // Out of sight while moving, and it was last seen at that edge of the picture: it
        // most likely carried on that way. Look further that way. (The edge comes from the
        // image itself, so a wrong speed estimate alone can't trigger this.)
        searched = true;
        motion_nudge(AXIS_PAN, st.exit_dir * kSearchDeg);
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

void tracker_pause(void)
{
    // Keeps the latch, the class and where it was seen; only the speed is stale.
    kf_on = false;
    switch_dir = 0;
}

bool tracker_latched(void) { return latched; }

void tracker_lose_target(void)
{
    latched = false;
    miss_ms = 0;
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
