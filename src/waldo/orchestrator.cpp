#include "orchestrator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion.h"
#include "tracker.h"

namespace {

constexpr uint32_t kLostToSentryMs = 5000;   // no target this long while tracking -> sentry
constexpr int kSentryDetectEvery = 4;        // frames
constexpr float kNudgeDeg = 10.0f;
constexpr float kParkTilt = -90.0f;

// Motion detection: mean absolute difference per 8x8 cell between consecutive frames.
constexpr int kCell = 8;
constexpr int kCellThresh = 18;              // grey levels
constexpr int kMinCells = 3;
constexpr uint32_t kSettleMs = 600;          // ignore motion this long after the turret moved

enum orch_state state = ORCH_SENTRY;
uint32_t state_since, last_target_ms, last_move_ms, frames_in_state;
uint8_t prev[CAM_MAX_W * CAM_MAX_H];
bool have_prev;
uint16_t prev_w, prev_h;
uint32_t n_motion;

void enter(enum orch_state s, uint32_t now)
{
    if (s == state) return;
    printf("orch: %s -> %s\n", orch_state_name(state), orch_state_name(s));
    state = s;
    state_since = now;
    frames_in_state = 0;
    if (s == ORCH_PARKED) {
        motion_set(AXIS_PAN, 0);
        motion_set(AXIS_TILT, kParkTilt);
        last_move_ms = now;
    } else if (s == ORCH_SENTRY) {
        motion_set(AXIS_PAN, 0);
        motion_set(AXIS_TILT, 0);
        last_move_ms = now;
    } else if (s == ORCH_HOLD) {
        // Freeze where the turret is now, not where it was heading.
        motion_set(AXIS_PAN, motion_pos(AXIS_PAN));
        motion_set(AXIS_TILT, motion_pos(AXIS_TILT));
    }
    have_prev = false;
}

bool turret_moving()
{
    return fabsf(motion_pos(AXIS_PAN) - motion_target(AXIS_PAN)) > 0.5f ||
           fabsf(motion_pos(AXIS_TILT) - motion_target(AXIS_TILT)) > 0.5f;
}

// Returns true (and the centre of the changed cells) when enough of the picture changed.
bool motion_seen(const struct cam_frame *f, uint32_t now, float *mx, float *my)
{
    const int w = f->width, h = f->height;
    bool hit = false;
    if (have_prev && prev_w == w && prev_h == h && now - last_move_ms > kSettleMs && !turret_moving()) {
        int cells = 0;
        float sx = 0, sy = 0;
        for (int cy = 0; cy + kCell <= h; cy += kCell) {
            for (int cx = 0; cx + kCell <= w; cx += kCell) {
                int d = 0;
                for (int y = cy; y < cy + kCell; y++) {
                    const uint8_t *a = f->pix + y * w + cx, *b = prev + y * w + cx;
                    for (int x = 0; x < kCell; x++) d += abs(a[x] - b[x]);
                }
                if (d > kCellThresh * kCell * kCell) {
                    cells++;
                    sx += cx + kCell / 2;
                    sy += cy + kCell / 2;
                }
            }
        }
        if (cells >= kMinCells) {
            *mx = sx / cells;
            *my = sy / cells;
            hit = true;
        }
    }
    memcpy(prev, f->pix, (size_t)w * h);
    prev_w = (uint16_t)w;
    prev_h = (uint16_t)h;
    have_prev = true;
    return hit;
}

}  // namespace

const char *orch_state_name(enum orch_state s)
{
    static const char *const names[] = {"PARKED", "SENTRY", "TRACK", "HOLD"};
    return names[s];
}

enum orch_state orch_state(void) { return state; }

void orch_init(uint32_t now)
{
    state = ORCH_SENTRY;
    state_since = last_target_ms = now;
    last_move_ms = now;
}

void orch_frame(const struct cam_frame *f, uint32_t now, struct det_box *boxes, int max, int *n_out)
{
    *n_out = -1;
    frames_in_state++;
    if (state == ORCH_PARKED) return;

    if (state == ORCH_SENTRY) {
        float mx, my;
        if (motion_seen(f, now, &mx, &my)) {
            n_motion++;
            // Turn toward the movement; the detector then finds what moved.
            struct det_box m = {(int)mx - 4, (int)my - 4, 8, 8, 1.0f};
            tracker_update(&m, 1, f->width, f->height, now);
            last_move_ms = now;
            printf("orch: motion at %.0f,%.0f\n", (double)mx, (double)my);
        }
        if (frames_in_state % kSentryDetectEvery != 0) return;
    }

    const int n = detect(f, boxes, max);
    *n_out = n;
    if (state == ORCH_HOLD) return;   // looks, doesn't move

    if (n > 0) {
        last_target_ms = now;
        if (state == ORCH_SENTRY) enter(ORCH_TRACK, now);
        tracker_update(boxes, n, f->width, f->height, now);
        last_move_ms = now;
    } else if (state == ORCH_TRACK) {
        tracker_update(boxes, 0, f->width, f->height, now);
        if (now - last_target_ms > kLostToSentryMs) enter(ORCH_SENTRY, now);
    }
}

void orch_word(const char *w, uint32_t now)
{
    printf("orch: heard \"%s\" in %s\n", w, orch_state_name(state));
    if (!strcmp(w, "off")) {
        enter(ORCH_PARKED, now);
    } else if (!strcmp(w, "on") || !strcmp(w, "go")) {
        if (state == ORCH_PARKED || state == ORCH_HOLD) enter(ORCH_SENTRY, now);
    } else if (!strcmp(w, "stop")) {
        if (state != ORCH_PARKED) enter(ORCH_HOLD, now);
    } else if (!strcmp(w, "no")) {
        if (state == ORCH_TRACK) tracker_switch(TRACK_SWITCH_ANY);
    } else {
        int dir = !strcmp(w, "left") ? TRACK_SWITCH_LEFT : !strcmp(w, "right") ? TRACK_SWITCH_RIGHT
                : !strcmp(w, "up") ? TRACK_SWITCH_UP : !strcmp(w, "down") ? TRACK_SWITCH_DOWN : 0;
        if (!dir || state == ORCH_PARKED) return;
        if (state == ORCH_TRACK) {
            tracker_switch(dir);
        } else {
            // +pan turns left, +tilt points up
            if (dir == TRACK_SWITCH_LEFT) motion_nudge(AXIS_PAN, kNudgeDeg);
            if (dir == TRACK_SWITCH_RIGHT) motion_nudge(AXIS_PAN, -kNudgeDeg);
            if (dir == TRACK_SWITCH_UP) motion_nudge(AXIS_TILT, kNudgeDeg);
            if (dir == TRACK_SWITCH_DOWN) motion_nudge(AXIS_TILT, -kNudgeDeg);
            last_move_ms = now;
        }
    }
}

void orch_print(uint32_t now)
{
    printf("orch: %s for %lu s, %lu motion events\n", orch_state_name(state),
           (unsigned long)((now - state_since) / 1000), (unsigned long)n_motion);
}
