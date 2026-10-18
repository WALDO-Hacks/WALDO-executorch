// Renders what the LCD shows for a camera JPEG, through the firmware's own code: JPEG
// decode, rotation (like camlink_rx.cpp), view.cpp and the board's display.c.
//   test_view <frame.jpg> <rotate> <state 0-3> <out.ppm> [x y w h ...]   (boxes in the
//   rotated grayscale frame's pixels, the first one the tracked target)
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "jpeg_frame.h"
#include "view.h"

extern "C" void display_host_save(const char *path);

// The detector's class names, for the box labels (the model's list, as in yolo_detector.cpp).
#include "../../src/yolo/waldo_yolo_classes.h"
const char *detect_class_name(int cls) { return cls >= 0 && cls < 28 ? waldo::kClassNames[cls] : "motion"; }

// The orchestrator's state names (its other code isn't needed for drawing).
const char *orch_state_name(enum orch_state s)
{
    static const char *const names[] = {"PARKED", "SENTRY", "TRACK", "HOLD"};
    return names[s];
}

int main(int argc, char **argv)
{
    if (argc < 5) {
        fprintf(stderr, "usage: %s frame.jpg rotate state out.ppm [x y w h ...]\n", argv[0]);
        return 2;
    }
    FILE *fi = fopen(argv[1], "rb");
    static uint8_t jpg[65536];
    const size_t len = fread(jpg, 1, sizeof jpg, fi);
    fclose(fi);

    static uint8_t rgb[320 * 240 * 3], rgb_rot[320 * 240 * 3], gray[160 * 120];
    static cam_frame f;
    uint16_t w, h;
    if (jpeg_frame_decode(jpg, (uint32_t)len, rgb, gray, &w, &h) != 0) {
        fprintf(stderr, "decode failed\n");
        return 1;
    }
    const int rot = atoi(argv[2]);
    frame_rotate(rgb, rgb_rot, w, h, 3, rot, &f.rgb_w, &f.rgb_h);
    frame_rotate(gray, f.pix, w / 2, h / 2, 1, rot, &f.width, &f.height);
    f.rgb = rgb_rot;

    std::vector<det_box> boxes;
    for (int i = 5; i + 3 < argc; i += 4)
        boxes.push_back({atoi(argv[i]), atoi(argv[i + 1]), atoi(argv[i + 2]), atoi(argv[i + 3]), 1.0f});
    const enum orch_state st = (enum orch_state)atoi(argv[3]);
    waldo_view_draw(&f, boxes.data(), (int)boxes.size(), 0, st);
    const view_status vs = {st, "person", "left", 500, 18.4f, 21100, 47300, st == ORCH_TRACK, true, true, 9.6f};
    waldo_view_status(&vs);
    display_host_save(argv[4]);
    printf("frame %ux%u colour %ux%u -> %s\n", f.width, f.height, f.rgb_w, f.rgb_h, argv[4]);
    return 0;
}
