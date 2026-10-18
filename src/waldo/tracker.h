// Turns detection boxes into turret motion. The camera rides on the tilt head: each frame
// turns the turret part of the way to centring the target, and between frames it keeps
// turning at the target's estimated speed (tracker.cpp has the details).
#pragma once
#include <stdint.h>
#include "detector.h"

#define DET_MOTION (-3)   // a box around movement, from the orchestrator
#define TRACK_PEOPLE (-2) // the default target class: person

// Steers toward the target among boxes: the one it was following, else the biggest of the
// target class (people until a switch picks something else). Returns true if a
// target was among the boxes. A box of class DET_MOTION is always a target.
bool tracker_update(const struct det_box *boxes, int n, int img_w, int img_h, uint32_t now_ms);
// Call every loop: records where the turret points, and while `following` turns with the
// target between frames, coasts through short misses and searches the way a lost target went.
void tracker_tick(uint32_t now_ms, bool following);
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
// Drop the current target (lost, or the turret moves on) but keep following its class.
void tracker_lose_target(void);
// Follow this YOLO class from now on (0 = people), e.g. a "track cup" command. The class
// stays until another one is chosen: tracker_switch(), this, or tracker_reset_target().
void tracker_set_target_class(int cls);
// Class being followed (detect_class_name), for the console.
int tracker_target_class(void);
