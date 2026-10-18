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

// Patrol (SENTRY after a quiet spell at home): move, settle, look, along a Z of stops
// (left to right at tilt 0, back right to left at tilt 20). Stops are 35 degrees apart:
// the rotated camera sees about 45 degrees across, so neighbouring views overlap.
// Each stop remembers a coarse picture of itself; YOLO runs only where the picture
// changed since the last lap, where something moves, on a first visit, or every few laps.
constexpr uint32_t kPatrolAfterMs = 4000;    // quiet at home this long -> start patrolling
constexpr uint32_t kLookMs = 700;            // time spent looking at each stop
constexpr int kRecheckLaps = 3;              // YOLO at every stop at least every 3 laps
constexpr int kInvestigateFrames = 6;        // detector frames after motion during a look
constexpr int kBgThresh = 20;                // grey levels per cell
constexpr int kBgMinCells = 3;
struct stop_t { float pan, tilt; };
constexpr stop_t kStops[] = {{70, 0}, {35, 0}, {0, 0}, {-35, 0}, {-70, 0},
                             {-70, 20}, {-35, 20}, {0, 20}, {35, 20}, {70, 20}};
constexpr int kNumStops = sizeof(kStops) / sizeof(kStops[0]);
constexpr int kBgCells = (CAM_MAX_W / kCell) * (CAM_MAX_H / kCell);   // 300

enum patrol_phase { P_HOME, P_MOVE, P_SETTLE, P_LOOK };
struct {
    patrol_phase phase = P_HOME;
    int stop, lap, investigate;
    uint32_t t;
    bool run_yolo;
    uint8_t bg[kNumStops][kBgCells];
    bool bg_valid[kNumStops];
    int laps_since_yolo[kNumStops];
    uint32_t looks, yolo_looks;
} patrol;

bool hold_target = true;                     // a lost target stays the target (see orchestrator.h)
enum orch_state state = ORCH_SENTRY;
enum orch_state before_hold = ORCH_SENTRY;   // "go" after "stop" goes back to it
uint32_t state_since, last_target_ms, last_move_ms, frames_in_state;
uint8_t prev[CAM_MAX_W * CAM_MAX_H];
bool have_prev;
uint16_t prev_w, prev_h;
uint32_t n_motion;

void enter(enum orch_state s, uint32_t now)
{
    if (s == state) return;
    printf("orch: %s -> %s\n", orch_state_name(state), orch_state_name(s));
    const enum orch_state state_before_enter = state;
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
        // Back to looking for people, unless a target was chosen and is to be kept (hold target):
        // only a fresh start from PARKED resets that.
        if (!hold_target || state_before_enter == ORCH_PARKED) tracker_reset_target();
        last_move_ms = now;
        patrol.phase = P_HOME;
        patrol.t = now;
        patrol.investigate = 0;
    } else if (s == ORCH_HOLD) {
        before_hold = state_before_enter;
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

// Coarse picture: the mean of each 8x8 cell.
int cells_of(const struct cam_frame *f, uint8_t *out, int *cw, int *ch)
{
    *cw = f->width / kCell;
    *ch = f->height / kCell;
    for (int cy = 0; cy < *ch; cy++)
        for (int cx = 0; cx < *cw; cx++) {
            int sum = 0;
            for (int y = 0; y < kCell; y++) {
                const uint8_t *row = f->pix + (cy * kCell + y) * f->width + cx * kCell;
                for (int x = 0; x < kCell; x++) sum += row[x];
            }
            out[cy * *cw + cx] = (uint8_t)(sum / (kCell * kCell));
        }
    return *cw * *ch;
}

// Changed cells between this stop's last picture and now. A servo never lands on exactly
// the same spot twice, so each cell is compared with the best of its neighbours too.
int changed_cells(const uint8_t *a, const uint8_t *b, int cw, int ch)
{
    int n = 0;
    for (int y = 0; y < ch; y++)
        for (int x = 0; x < cw; x++) {
            int best = 255;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    const int yy = y + dy, xx = x + dx;
                    if (yy < 0 || yy >= ch || xx < 0 || xx >= cw) continue;
                    const int d = abs(a[y * cw + x] - b[yy * cw + xx]);
                    if (d < best) best = d;
                }
            if (best > kBgThresh) n++;
        }
    return n;
}

void go_to_stop(int i, uint32_t now)
{
    patrol.stop = i;
    patrol.phase = P_MOVE;
    patrol.t = now;
    motion_set(AXIS_PAN, kStops[i].pan);
    motion_set(AXIS_TILT, kStops[i].tilt);
    last_move_ms = now;
    have_prev = false;
}

// One SENTRY frame: home, or a patrol step. Returns true if the detector should run on it.
bool sentry_step(const struct cam_frame *f, uint32_t now)
{
    float mx, my;
    const bool moved = motion_seen(f, now, &mx, &my);
    if (moved) {
        n_motion++;
        // Turn toward the movement; the detector then looks at what moved.
        struct det_box m = {(int)mx - 4, (int)my - 4, 8, 8, 1.0f, DET_MOTION};
        tracker_update(&m, 1, f->width, f->height, now);
        last_move_ms = now;
        patrol.investigate = kInvestigateFrames;
        printf("orch: motion at %.0f,%.0f\n", (double)mx, (double)my);
    }
    if (patrol.investigate > 0) {   // something moved: look properly before going on
        patrol.investigate--;
        if (patrol.investigate == 0 && patrol.phase != P_HOME) go_to_stop((patrol.stop + 1) % kNumStops, now);
        return true;
    }

    switch (patrol.phase) {
    case P_HOME:
        if (now - patrol.t >= kPatrolAfterMs) {
            patrol.lap = 0;
            go_to_stop(0, now);
            return false;
        }
        return frames_in_state % kSentryDetectEvery == 0;
    case P_MOVE:
        last_move_ms = now;
        if (!turret_moving()) {
            patrol.phase = P_SETTLE;
            patrol.t = now;
        }
        return false;
    case P_SETTLE:
        if (now - patrol.t < kSettleMs) return false;
        {
            // Compare with this stop's picture from the last lap.
            uint8_t cur[kBgCells];
            int cw, ch;
            cells_of(f, cur, &cw, &ch);
            const int i = patrol.stop;
            const int changed = patrol.bg_valid[i] ? changed_cells(cur, patrol.bg[i], cw, ch) : -1;
            patrol.run_yolo = changed < 0 || changed >= kBgMinCells || patrol.laps_since_yolo[i] >= kRecheckLaps;
            memcpy(patrol.bg[i], cur, (size_t)cw * ch);
            patrol.bg_valid[i] = true;
            patrol.laps_since_yolo[i] = patrol.run_yolo ? 0 : patrol.laps_since_yolo[i] + 1;
            patrol.looks++;
            patrol.yolo_looks += patrol.run_yolo;
            if (changed >= kBgMinCells) printf("orch: stop %d changed (%d cells)\n", i, changed);
            patrol.phase = P_LOOK;
            patrol.t = now;
            return patrol.run_yolo;
        }
    case P_LOOK:
        if (now - patrol.t >= kLookMs) {
            const int next = (patrol.stop + 1) % kNumStops;
            if (next == 0) patrol.lap++;
            go_to_stop(next, now);
        }
        return false;
    }
    return false;
}

}  // namespace

const char *orch_detail(void)
{
    static char buf[24];
    if (state != ORCH_SENTRY || patrol.phase == P_HOME) return nullptr;
    snprintf(buf, sizeof buf, "scan %d/%d", patrol.stop + 1, kNumStops);
    return buf;
}

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
    patrol.phase = P_HOME;
    patrol.t = now;
    patrol.investigate = 0;
}

void orch_frame(const struct cam_frame *f, uint32_t now, struct det_box *boxes, int max, int *n_out)
{
    *n_out = -1;
    frames_in_state++;
    if (state == ORCH_PARKED) return;

    if (state == ORCH_SENTRY && !sentry_step(f, now)) return;

    const int n = detect(f, boxes, max);
    *n_out = n;
    if (state == ORCH_HOLD) return;   // looks, doesn't move

    // Only a box of the target class counts (people, until "left"/"right" picks an
    // object): a chair in the background must not keep WALDO busy.
    if (tracker_update(boxes, n, f->width, f->height, now)) {
        last_target_ms = now;
        last_move_ms = now;
        if (state == ORCH_SENTRY) enter(ORCH_TRACK, now);
    } else if (state == ORCH_TRACK && !hold_target && now - last_target_ms > kLostToSentryMs) {
        enter(ORCH_SENTRY, now);
    }
}

void orch_tick(uint32_t now)
{
    // Frames stopped (camera unplugged) while tracking: orch_frame() never runs, so give up here.
    if (state == ORCH_TRACK && !hold_target && now - last_target_ms > kLostToSentryMs + 1000) enter(ORCH_SENTRY, now);
}

void orch_word(const char *w, uint32_t now)
{
    printf("orch: heard \"%s\" in %s\n", w, orch_state_name(state));
    if (!strcmp(w, "off")) {
        enter(ORCH_PARKED, now);
    } else if (!strcmp(w, "on") || !strcmp(w, "go")) {
        if (state == ORCH_HOLD && before_hold == ORCH_TRACK) {
            enter(ORCH_TRACK, now);   // resume following the same target, from where it is
            last_target_ms = now;
        } else if (state == ORCH_PARKED || state == ORCH_HOLD) {
            enter(ORCH_SENTRY, now);
        }
    } else if (!strcmp(w, "stop")) {
        if (state != ORCH_PARKED) enter(ORCH_HOLD, now);
    } else if (!strcmp(w, "no")) {
        tracker_reset_target();   // drop what it follows, back to people
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

void orch_set_hold_target(bool on) { hold_target = on; }
bool orch_hold_target(void) { return hold_target; }

void orch_track_class(int cls, uint32_t now)
{
    printf("orch: track %s in %s\n", detect_class_name(cls), orch_state_name(state));
    if (state == ORCH_PARKED) return;
    tracker_set_target_class(cls);
    last_target_ms = now;
}

void orch_print(uint32_t now)
{
    printf("orch: %s for %lu s, following %s, %lu motion events, patrol: YOLO at %lu of %lu stops looked at\n",
           orch_state_name(state), (unsigned long)((now - state_since) / 1000),
           detect_class_name(tracker_target_class()), (unsigned long)n_motion,
           (unsigned long)patrol.yolo_looks, (unsigned long)patrol.looks);
}
