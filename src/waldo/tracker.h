// Turns detection boxes into turret motion. The camera rides on the tilt head, so each
// frame nudges the turret a fraction of the way toward centring the target.
#pragma once
#include <stdint.h>
#include "detector.h"

#define DET_MOTION (-3)   // a box around movement, from the orchestrator
#define TRACK_PEOPLE (-2) // the default target class: person

// Steers toward the target among boxes: the one it was following, else the biggest of the
// target class (people until a switch picks something else). Returns true if a
// target was among the boxes. A box of class DET_MOTION is always a target.
bool tracker_update(const struct det_box *boxes, int n, int img_w, int img_h, uint32_t now_ms);
void tracker_print(void);
// Index (in the last boxes passed to tracker_update) of the target being followed, or -1.
int tracker_target_index(void);

// Switch to another target at the next update: the nearest box in that direction in the
// image (the turret's left is the image's left), of any class: the tracker then follows
// that class. ANY = the nearest other box.
enum { TRACK_SWITCH_ANY = 1, TRACK_SWITCH_LEFT, TRACK_SWITCH_RIGHT, TRACK_SWITCH_UP, TRACK_SWITCH_DOWN };
void tracker_switch(int dir);
// Drop the current target and go back to following people.
void tracker_reset_target(void);
// Class being followed (detect_class_name), for the console.
int tracker_target_class(void);
