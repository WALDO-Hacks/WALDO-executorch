#include "tracker.h"

#include <math.h>
#include <stdio.h>

#include "motion.h"

static struct {
    float fov_long = 60;        // ESP32 OV3660: degrees across the sensor's long side
    float kp = 0.2f;            // fraction of the angle error corrected per frame; 0.35 (and 0.30 on
                                // pan) overshot back and forth on the board
    float deadband = 3.0f;      // degrees
    int sign_pan = 1, sign_tilt = 1;
} cfg;

static int target_index = -1;
static int switch_dir;   // pending tracker_switch()
static uint32_t switch_ms;
constexpr uint32_t kSwitchMs = 1500;   // a switch that finds nothing in this long is dropped
static int target_cls = TRACK_PEOPLE;
// A box away from the current target must win kConfirmFrames frames in a row before the
// tracker moves to it, while the target was seen in the last kConfirmMs. Until then the
// turret holds: a one-frame miss or a false box doesn't pull it away.
constexpr int kConfirmFrames = 3;
constexpr uint32_t kConfirmMs = 1000;
static int confirm_n;

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
    int cls = DET_MOTION;   // class of the target; a movement box is never confirmed against
    float err_pan, err_tilt;
} st;

bool tracker_update(const struct det_box *b, int n, int img_w, int img_h, uint32_t now)
{
    // Stay on the target we were following if it's still there, else take the biggest
    // box of the target class.
    int pick = -1;
    float best = -1;
    bool switched = false;   // a voice command picked it: no confirmation
    // The target was seen just now, or (a real target) within kConfirmMs: it keeps its bonus.
    const bool recent = st.has_target || (st.cls != DET_MOTION && now - st.last_seen_ms < kConfirmMs);
    for (int i = 0; i < n; i++) {
        if (!is_target_class(b[i].cls)) continue;
        float cx = b[i].x + b[i].w / 2.0f, cy = b[i].y + b[i].h / 2.0f;
        float score = (float)b[i].w * b[i].h;
        if (recent && hypotf(cx - st.cx, cy - st.cy) < 1.5f * st.w + 8) score *= 4;
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
            printf("track: locked onto a %s\n", detect_class_name(target_cls));
            switch_dir = 0;
            switched = true;
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
            printf("track: switched to a %s\n", detect_class_name(target_cls));
            switched = true;
        } else {
            printf("track: no other target that way\n");
        }
        switch_dir = 0;
    }
    // The pick is somewhere else than the target (and no switch was asked for): only after it
    // has won kConfirmFrames frames in a row.
    if (pick >= 0 && !switched && recent && st.cls != DET_MOTION && b[pick].cls != DET_MOTION &&
        hypotf(b[pick].x + b[pick].w / 2.0f - st.cx, b[pick].y + b[pick].h / 2.0f - st.cy) >= 1.5f * st.w + 8) {
        if (++confirm_n < kConfirmFrames) pick = -1;
    }
    if (pick < 0) {
        target_index = -1;
        st.has_target = false;
        return false;   // what to do about a lost target is the orchestrator's call
    }
    confirm_n = 0;
    target_index = pick;
    st.cx = b[pick].x + b[pick].w / 2.0f;
    // Aim at the centre of the box. Not lower (the chest): a person cut off by the bottom of
    // the picture (sitting at a desk, standing close) keeps a low aim point below the centre
    // however far the head tilts down, so it ran to the tilt floor (board, 2026-10-18). The
    // centre settles with the head near the top of the picture.
    st.cy = b[pick].y + b[pick].h * 0.5f;
    st.w = b[pick].w;
    st.cls = b[pick].cls;
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
    if (fabsf(st.err_pan) > cfg.deadband) motion_nudge(AXIS_PAN, cfg.kp * st.err_pan);
    if (fabsf(st.err_tilt) > cfg.deadband) motion_nudge(AXIS_TILT, cfg.kp * st.err_tilt);
    return true;
}

void tracker_print(void)
{
    printf("track: %s, seen %lu, error pan %+.1f tilt %+.1f, turret pan %.1f tilt %.1f\n",
           st.has_target ? "TARGET" : (st.at_home ? "home" : "lost"), (unsigned long)st.seen,
           (double)st.err_pan, (double)st.err_tilt, (double)motion_pos(AXIS_PAN),
           (double)motion_pos(AXIS_TILT));
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
    st.cls = DET_MOTION;   // whoever is found next is taken at once
    confirm_n = 0;
    target_index = -1;
    switch_dir = 0;
}

void tracker_pause(void) { switch_dir = 0; }

void tracker_set_target_class(int cls)
{
    target_cls = cls == 0 ? TRACK_PEOPLE : cls;
    tracker_lose_target();
}

int tracker_target_class(void) { return target_cls == TRACK_PEOPLE ? 0 : target_cls; }
