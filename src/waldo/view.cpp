#include "view.h"

#include <stdio.h>
#include <string.h>

#include "board_display.h"

void waldo_view_draw(const struct cam_frame *f, const struct det_box *boxes, int n, int target,
                     enum orch_state state)
{
    const bool colour = f->rgb != nullptr;
    const int k = colour ? 2 : 1;   // boxes are in grayscale-frame pixels; colour is twice the size
    if ((colour ? board_display_image(f->rgb, f->rgb_w, f->rgb_h, 3)
                : board_display_image(f->pix, f->width, f->height, 1)) != 0)
        return;
    // The target green (thick); other people yellow; objects cyan.
    for (int i = 0; i < n; i++) {
        const bool t = i == target && state == ORCH_TRACK;
        const bool person = boxes[i].cls == 0;
        const uint8_t r = t ? 0 : person ? 255 : 0, g = t ? 255 : person ? 200 : 200, b = t ? 0 : person ? 0 : 255;
        board_display_box(k * boxes[i].x, k * boxes[i].y, k * boxes[i].w, k * boxes[i].h, r, g, b, t ? 3 : 2);
    }
    // Centre mark: the tracker drives the target onto it.
    board_display_box(k * f->width / 2 - 2, k * f->height / 2 - 2, 5, 5, 255, 255, 255, 1);
}

// State colours: red PARKED, blue SENTRY, green TRACK, yellow HOLD.
static const uint8_t kStateColour[4][3] = {{230, 40, 40}, {60, 140, 255}, {40, 220, 40}, {255, 200, 0}};

void waldo_view_status(const struct view_status *s)
{
    static char last_top[128], last_bottom[160];
    char top[48], eyes[48], ears[48], heard[40], perf[48], key_top[128], key_bottom[160];
    snprintf(top, sizeof top, "%s  %s", orch_state_name(s->state),
             s->state == ORCH_PARKED ? "" : s->following);
    // EYES: the big core. YOLO on its NPU, or the cheap motion watch on the CPU.
    const bool parked = s->state == ORCH_PARKED;
    if (parked) snprintf(eyes, sizeof eyes, "EYES  off (parked)");
    else if (s->yolo_active)
        snprintf(eyes, sizeof eyes, "EYES  M55-HP+U85  YOLO %lu ms", (unsigned long)((s->npu_us + 500) / 1000));
    else snprintf(eyes, sizeof eyes, "EYES  M55-HP  motion watch");
    // EARS: the small core in the two-brain build.
    if (!s->two_brain) snprintf(ears, sizeof ears, "ONE BRAIN  M55-HP hears too");
    else if (s->ears_ok) snprintf(ears, sizeof ears, "EARS  M55-HE+U55  %.0f%% busy", (double)s->ears_busy);
    else snprintf(ears, sizeof ears, "EARS  M55-HE  not listening");
    if (s->heard) snprintf(heard, sizeof heard, "heard \"%s\"", s->heard);
    else snprintf(heard, sizeof heard, "say: go stop left right yes off");
    snprintf(perf, sizeof perf, "%4.1f fps   frame %2lu ms", (double)s->fps,
             (unsigned long)((s->frame_us + 500) / 1000));
    const bool fresh = s->heard && s->heard_age_ms < 2000;   // a new word stands out for 2 s
    snprintf(key_top, sizeof key_top, "%s|%s|%d", top, eyes, s->yolo_active);
    snprintf(key_bottom, sizeof key_bottom, "%s|%s|%s|%d|%d", ears, heard, perf, fresh, s->ears_ok);

    const int w = BOARD_DISPLAY_WIDTH, h = BOARD_DISPLAY_HEIGHT;
    if (strcmp(key_top, last_top) != 0) {
        const uint8_t *c = kStateColour[s->state];
        board_display_fill(0, 0, w, 80, 0, 0, 0);
        board_display_text(12, 4, top, c[0], c[1], c[2], 3);
        // YOLO running: bright green; motion watch: dim blue; parked: grey.
        if (parked) board_display_text(12, 46, eyes, 120, 120, 120, 2);
        else if (s->yolo_active) board_display_text(12, 46, eyes, 60, 255, 60, 2);
        else board_display_text(12, 46, eyes, 110, 150, 220, 2);
        board_display_fill(0, 74, w, 6, c[0], c[1], c[2]);
        strcpy(last_top, key_top);
    }
    if (strcmp(key_bottom, last_bottom) != 0) {
        board_display_fill(0, h - 80, w, 80, 0, 0, 0);
        // EARS orange while the HE listens, red if it stopped; one brain grey.
        if (!s->two_brain) board_display_text(12, h - 78, ears, 150, 150, 150, 2);
        else if (s->ears_ok) board_display_text(12, h - 78, ears, 255, 160, 40, 2);
        else board_display_text(12, h - 78, ears, 255, 60, 60, 2);
        board_display_text(12, h - 52, heard, fresh ? 255 : 150, fresh ? 220 : 150, fresh ? 0 : 150, 2);
        board_display_text(12, h - 26, perf, 200, 200, 200, 2);
        strcpy(last_bottom, key_bottom);
    }
}
