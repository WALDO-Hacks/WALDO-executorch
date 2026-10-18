#include "tracker.h"

#include <math.h>
#include <stdio.h>

#include "motion.h"

static struct {
    float hfov = 60;            // ESP32 OV3660 at 160x120, horizontal, degrees
    float kp = 0.35f;           // fraction of the angle error corrected per frame
    float deadband = 1.5f;      // degrees
    int sign_pan = 1, sign_tilt = 1;
    uint32_t home_after_ms = 5000;
} cfg;

static struct {
    bool has_target, at_home = true;
    float cx, cy, w;
    uint32_t last_seen_ms, seen;
    float err_pan, err_tilt;
} st;

void tracker_update(const struct det_box *b, int n, int img_w, int img_h, uint32_t now)
{
    if (n == 0) {
        st.has_target = false;
        if (!st.at_home && now - st.last_seen_ms > cfg.home_after_ms) {
            // Look straight ahead again so the camera can find the next target.
            motion_set(AXIS_PAN, 0);
            motion_set(AXIS_TILT, 0);
            st.at_home = true;
        }
        return;
    }

    // Stay on the target we were following if it's still there, else take the biggest.
    int pick = 0;
    float best = -1;
    for (int i = 0; i < n; i++) {
        float cx = b[i].x + b[i].w / 2.0f, cy = b[i].y + b[i].h / 2.0f;
        float score = (float)b[i].w * b[i].h;
        if (st.has_target && hypotf(cx - st.cx, cy - st.cy) < 1.5f * st.w + 8) score *= 4;
        if (score > best) {
            best = score;
            pick = i;
        }
    }
    st.cx = b[pick].x + b[pick].w / 2.0f;
    st.cy = b[pick].y + b[pick].h / 2.0f;
    st.w = b[pick].w;
    st.has_target = true;
    st.at_home = false;
    st.last_seen_ms = now;
    st.seen++;

    // Angle of the target off the optical axis; +x right, +y down in the image.
    float vfov = cfg.hfov * img_h / img_w;
    float ex = (st.cx / img_w - 0.5f) * cfg.hfov;
    float ey = (st.cy / img_h - 0.5f) * vfov;
    // +pan turns left and +tilt points up, so a target right of / below centre needs negative.
    st.err_pan = -ex * cfg.sign_pan;
    st.err_tilt = -ey * cfg.sign_tilt;
    if (fabsf(st.err_pan) > cfg.deadband) motion_nudge(AXIS_PAN, cfg.kp * st.err_pan);
    if (fabsf(st.err_tilt) > cfg.deadband) motion_nudge(AXIS_TILT, cfg.kp * st.err_tilt);
}

void tracker_print(void)
{
    printf("track: %s, seen %lu, error pan %+.1f tilt %+.1f, turret pan %.1f tilt %.1f\n",
           st.has_target ? "TARGET" : (st.at_home ? "home" : "lost"), (unsigned long)st.seen,
           (double)st.err_pan, (double)st.err_tilt, (double)motion_pos(AXIS_PAN),
           (double)motion_pos(AXIS_TILT));
}
