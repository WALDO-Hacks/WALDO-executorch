// WALDO's orchestrator: one state machine that decides what the turret does with each
// camera frame and each voice command. The tracker steers; this decides when and at what.
//
//   PARKED  head down, vision off; only "on" / "go" wakes it
//   SENTRY  waiting: at home, motion detection every frame (CPU, no model) and the detector
//           every 2nd frame; after 4 quiet seconds it patrols a Z of 10 stops (move, settle,
//           look) and runs the detector only where the view changed since the last lap or
//           something moves (plus every 3rd lap); motion or a target -> TRACK
//   TRACK   detector every frame, the tracker follows the target (and coasts/searches
//           through short losses); lost: holds, looks left and right, then after 8 s -> SENTRY, patrolling on from the nearest
//           stop and still looking for the same kind of thing
//   HOLD    "stop": the turret freezes but keeps looking; "go" resumes
//
// Voice: on/go wake or resume, stop holds, off parks; left/right/up/down nudge the turret,
// or while tracking switch to the nearest thing in that direction (a person or an object,
// which it then follows); yes locks onto whatever is in the middle of the picture and
// follows that kind of thing; no drops the target and goes back to people.
#pragma once
#include <stdint.h>

#include "camlink_rx.h"
#include "detector.h"

enum orch_state { ORCH_PARKED, ORCH_SENTRY, ORCH_TRACK, ORCH_HOLD };

void orch_init(uint32_t now_ms);
// A new camera frame: runs what the state needs. Fills boxes (at most max) with what the
// detector found (n_out, -1 if it didn't run this frame) for the screen.
void orch_frame(const struct cam_frame *f, uint32_t now_ms, struct det_box *boxes, int max, int *n_out);
// A recognised voice command (kws_label() text).
// Every main-loop pass: time-outs that must work without camera frames.
void orch_tick(uint32_t now);
void orch_word(const char *word, uint32_t now_ms);
// Track this YOLO class (a command by name, "track cup"; 0 = people). It is kept until
// another is chosen, also through "stop"/"go" and a lost target; "off" then "on" starts
// again with people. Ignored while PARKED. Nothing calls it by voice yet: the keyword
// spotter has no object words.
void orch_track_class(int cls, uint32_t now_ms);
// Wake word ("Hey Waldo"). When required (for the two-brain build, once the HE reports a
// wake detector), a command acts only while listening: orch_wake() opens a 5 s window
// that every accepted command renews. "stop" and "off" always work. Not required by
// default: every heard word acts.
void orch_wake(uint32_t now_ms);
void orch_set_wake_required(bool on);
bool orch_listening(uint32_t now_ms);
enum orch_state orch_state(void);
const char *orch_state_name(enum orch_state s);
// What SENTRY is doing ("scan 3/10"), or NULL.
const char *orch_detail(void);
void orch_print(uint32_t now_ms);
