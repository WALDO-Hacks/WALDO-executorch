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
    static char last_top[48], last_bottom[96];
    char top[48], heard[40], perf[56], bottom[96];
    snprintf(top, sizeof top, "%s  %s", orch_state_name(s->state),
             s->state == ORCH_PARKED ? "" : s->following);
    if (s->heard) snprintf(heard, sizeof heard, "heard \"%s\"", s->heard);
    else snprintf(heard, sizeof heard, "say: go stop left right off");
    snprintf(perf, sizeof perf, "%4.1f fps   NPU %2lu ms   frame %2lu ms", (double)s->fps,
             (unsigned long)((s->npu_us + 500) / 1000), (unsigned long)((s->frame_us + 500) / 1000));
    const bool fresh = s->heard && s->heard_age_ms < 2000;   // a new word stands out for 2 s
    snprintf(bottom, sizeof bottom, "%s|%s|%d", heard, perf, fresh);

    const int w = BOARD_DISPLAY_W, h = BOARD_DISPLAY_H;
    if (strcmp(top, last_top) != 0) {
        const uint8_t *c = kStateColour[s->state];
        board_display_fill(0, 0, w, 80, 0, 0, 0);
        board_display_text(16, 14, top, c[0], c[1], c[2], 4);
        board_display_fill(0, 74, w, 6, c[0], c[1], c[2]);
        strcpy(last_top, top);
    }
    if (strcmp(bottom, last_bottom) != 0) {
        board_display_fill(0, h - 80, w, 80, 0, 0, 0);
        board_display_text(16, h - 72, heard, fresh ? 255 : 150, fresh ? 220 : 150, fresh ? 0 : 150, 3);
        board_display_text(16, h - 30, perf, 200, 200, 200, 2);
        strcpy(last_bottom, bottom);
    }
}
