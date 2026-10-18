#include "view.h"

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
    // State bar: red PARKED, blue SENTRY, green TRACK, yellow HOLD.
    static const uint8_t bar[4][3] = {{220, 0, 0}, {0, 90, 255}, {0, 220, 0}, {255, 200, 0}};
    const uint8_t *c = bar[state];
    // A solid stripe 16 panel pixels tall: the panel shows a colour frame at 2x, gray at 4x.
    board_display_box(0, 0, k * f->width, colour ? 8 : 4, c[0], c[1], c[2], 8);
}
