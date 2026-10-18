#include "tracker.h"

#include <math.h>
#include <stdio.h>

#include "motion.h"

static struct {
    float fov_long = 60;        // ESP32 OV3660: degrees across the sensor's long side
    float kp = 0.35f;           // fraction of the angle error corrected per frame
    float deadband = 1.5f;      // degrees
    int sign_pan = 1, sign_tilt = 1;
} cfg;

static int target_index = -1;
static int switch_dir;   // pending tracker_switch()
static int target_cls = TRACK_PEOPLE;

static bool is_target_class(int cls)
{
    if (cls == DET_MOTION) return true;
    if (target_cls == TRACK_PEOPLE) return cls == 0 || cls == DET_FACE;   // YOLO person, BlazeFace
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
    int pick = -1;
    float best = -1;
    for (int i = 0; i < n; i++) {
        if (!is_target_class(b[i].cls)) continue;
        float cx = b[i].x + b[i].w / 2.0f, cy = b[i].y + b[i].h / 2.0f;
        float score = (float)b[i].w * b[i].h;
        if (st.has_target && hypotf(cx - st.cx, cy - st.cy) < 1.5f * st.w + 8) score *= 4;
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
    st.cy = b[pick].y + b[pick].h / 2.0f;
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

void tracker_switch(int dir) { switch_dir = dir; }

void tracker_reset_target(void)
{
    target_cls = TRACK_PEOPLE;
    st.has_target = false;
    target_index = -1;
}

int tracker_target_class(void) { return target_cls == TRACK_PEOPLE ? 0 : target_cls; }
