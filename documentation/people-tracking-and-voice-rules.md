# People-only WALDO: plan and rulesets for latching, following and voice commands

Status: **PROPOSED**. Every number below is a starting value, not a measurement. Nothing here has been run on the board.
Code read for this doc: `src/waldo/orchestrator.cpp`, `src/waldo/tracker.cpp`, `he/wake_debounce.h`, `documentation/voice-commands.md` (branch `two-brain-waldo-integration`).
Not read: the partner's rewritten tracker on `origin/two-brain` (`06b024b`). Reconcile before implementing (Phase 0).

Scope change: the turret tracks **people only**. Objects, `orch_track_class` and "switch to a cup" are out of scope.

## What the code does today, and where it is fragile (from reading it)

| # | Finding | Where | Consequence |
|---|---|---|---|
| G1 | The detector's `score` is never used. Any `person` box above the detector's own threshold is a target | `tracker_update` | A poster or mannequin can latch |
| G2 | First sighting latches at once (SENTRY -> TRACK on one frame) | `orch_frame` | One false positive sends the turret chasing |
| G3 | Latch continuity is a x4 score bonus for boxes inside a gate, else box area wins | `tracker.cpp:100-108` | A person with at least 4x the area (2x the linear size) who is outside the gate steals the lock, e.g. someone walking between you and the camera |
| G4 | `tracker_switch` accepts any class | `tracker.cpp:116-142` | "left" can latch a chair or cup |
| G5 | With hold target on (default), a lost target never times out: the exit for "camera frames stopped" only runs when `!hold_target` | `orchestrator.cpp:317,326` | Camera unplugged or stalled while tracking = turret sits there forever, no error |
| G6 | `orch_listening()` exists but nothing shows it on the screen | grep | The operator cannot tell whether "Hey Waldo" worked |
| G7 | Safety words `stop` and `off` bypass the wake word at the normal 0.80 threshold | `orch_word` | Background talk that sounds like "off" parks the turret mid-demo (`stop` is fail-safe, `off` is not) |
| G8 | Without a running wake detector every word acts | `orch_set_wake_required` | In a noisy room the turret reacts to everything |
| G9 | `kFrameAgeMs = 160` is an estimate ("est. 150-200 ms") | `tracker.cpp:27` | The feed-forward and the gate depend on it. UNVERIFIED |
| G10 | Real-voice accuracy on the board is unmeasured; only synthetic voices (96 % clean, 80 % at 20 dB noise, N=260) | `voice-commands.md` | No false-accept rate exists at all |

## Tracking rulesets

IDs are stable so tests, commits and the partner's agent can cite them. "Test" = a host scenario in `tests/host/` (single target, run with `nice`).

### A. Acquire (SENTRY -> TRACK)

| ID | Rule | Starting value | Test |
|---|---|---|---|
| A1 | Eligible = class `person` **and** score >= S_min **and** box height >= H_min of frame height | S_min 0.45, H_min 12 % | box below either: never latches |
| A2 | Latch only after the person is seen in >= K of the last N detector frames, within the same gate | K=3, N=4 | single-frame ghost: no TRACK; steady person: TRACK within 4 frames |
| A3 | Motion (frame difference) only **turns the gaze** so the detector looks there. It never latches by itself | n/a | motion with no person: stays SENTRY |
| A4 | With several eligible people, latch the one maximising score x area; tie -> nearest the image centre. Deterministic | n/a | two equal boxes: same pick every run |

### B. Hold the latch (the "do not get stolen" rules)

| ID | Rule | Starting value | Test |
|---|---|---|---|
| B1 | A candidate **inside** the prediction gate always beats one outside it; among inside-gate candidates take the nearest to the prediction. Area never overrides the gate (replaces the x4 bonus, fixes G3) | n/a | big person crosses in front: lock stays on the original |
| B2 | A candidate outside the gate may take over only after the latched person has had no inside-gate match for M consecutive detector frames | M=3 | person leaves, another appears: takeover after M frames, not before |
| B3 | Occlusion: if the latched box vanishes while another overlaps its last box (IoU > 0.3), keep coasting on the filter; do not jump to the other | n/a | crossing scenario, both directions |
| B4 | Gate width grows with filter speed and time since last sighting (existing formula), capped so it never exceeds half the camera field of view | cap 30 deg | fast runner is kept; a person 60 deg away is never "the same" |
| B5 | Latch is single: other people are drawn on the LCD but never followed | n/a | 3 people, one lock |

### C. Follow

| ID | Rule | Starting value | Test |
|---|---|---|---|
| C1 | **Eye safety (hard rule):** the aim point is never above the box's vertical centre. Current 65 % down the box is kept | aim 0.65 | assert for every box shape; clipped boxes included |
| C2 | Keep the filter and gains (`kKp` 0.35, feed-forward 0.5, deadband 2 deg) until a replay shows a reason to change them | existing | no change without a log |
| C3 | At a mechanical limit (`motion` clamps to `min_deg`/`max_deg`), commanding further for > 1 s counts as "lost at the edge": stop pushing, do not wind up | 1 s | target walks out past the limit |
| C4 | Measure the real frame age (capture to turret command) on the board and replace the 160 ms guess (G9) | measured | log, not a unit test |

### D. Lost

| ID | Rule | Starting value | Test |
|---|---|---|---|
| D1 | Coast 0.7 s along the filter speed, then one search nudge up to 30 deg the way it went (existing) | existing | keep |
| D2 | After the search, hold the pose for T_hold waiting for the same person to return (inside the widened gate), then drop the latch and patrol | T_hold 8 s | **OPEN DECISION 1** below |
| D3 | If no camera frame arrives for 1 s: stop all motion, print an error. Independent of hold target (fixes G5) | 1 s | stop feeding frames while TRACK |
| D4 | Dropping the latch resets the filter and the target; patrol resumes from the nearest stop (existing) | existing | keep |

### E. Patrol (SENTRY)

| ID | Rule | Test |
|---|---|---|
| E1 | The patrol stops, timings and the "look only where the picture changed" logic stay as they are. They already look for people | keep |
| E2 | Nothing detected for a full lap of the Z: continue patrolling, no state change, no noise | keep |

## Voice rulesets

### F. What each word does (people-only)

| Word | PARKED | SENTRY | TRACK | HOLD |
|---|---|---|---|---|
| **stop** (always works, no wake word) | no effect | HOLD | HOLD | no effect |
| **off** (park) | no effect | park | park | park |
| **on** / **go** | wake to SENTRY | no effect | no effect | resume (TRACK on the same person if it was tracking) |
| **left/right/up/down** | ignored | nudge 10 deg | **latch the nearest other person that way**; none -> say so, keep the lock (new: replaces "switch to any class") | nudge 10 deg |
| **no** | no effect | no effect | **drop the latch, go patrol** | no effect |
| **yes** | log only | log only | log only | log only |

### G. Accepting a word (robustness rules)

| ID | Rule | Starting value | Test |
|---|---|---|---|
| V1 | Wake word required by default in the demo build (`wake_required`). **Fail closed:** if the HE wake detector is not running, only `stop` and `off` are accepted, not "everything" (fixes G8) | n/a | `test_brain` known-answer sequences |
| V2 | `stop` accepted at the current threshold (0.80 avg of 3 windows). It is fail-safe, so favour recall | 0.80 | existing |
| V3 | `off` needs more evidence than `stop`: score >= 0.90, **or** heard inside an open listening window (fixes G7) | 0.90 | `off` at 0.85 outside the window: ignored |
| V4 | One state-changing command per 500 ms; the same word is ignored for 1 s (existing) | 500 ms | burst of words |
| V5 | Listening window 5 s, renewed by each accepted command, **shown on the screen** (border or icon) and the accepted word flashes with its score (fixes G6) | 5 s | firmware change in `view` |
| V6 | Words invalid for the current state are logged and ignored (existing table). No hidden side effects | n/a | existing |
| V7 | Hypothesis: servo noise while moving lowers scores or adds false words. Log score vs servo speed first; only then decide whether to mute non-safety words while moving | HYPOTHESIS | log on board |
| V8 | Adding words (e.g. a spoken "follow me") needs retraining the 12-label model. Out of scope until the rules above are measured | n/a | n/a |

## Validation (so the claims are traceable)

Everything below is **exploratory** until a protocol is locked.

1. **Tracking replay.** Scripted trajectories fed to the real tracker and orchestrator code on the host: crossing, occlusion, appear/disappear, runner, edge exit, three people, ghost box. These are firmware-logic tests, not proof the board behaves the same.
2. **Voice on real people, on the board.** At least 5 speakers (PROPOSED), distances 0.5/1/2/3 m, three rooms (quiet, music, crowd). Report per-word recall and false accepts **per hour** of non-command speech (1 h minimum), each with a Wilson 95 % CI and N. Tune thresholds on some sessions, report on sessions not used for tuning.
3. **Proposed targets** (not measured, to be agreed): `stop` recall >= 95 %, other commands >= 85 %, wake false accepts <= 1 per hour, `off` false accepts <= 1 per 8 h.
4. **Board tracking sessions.** Log (CSV): frame time, boxes, latched index, turret pose, state, frame age. Same logs feed step 1.

## Plan

| Phase | Work | Blocks / notes |
|---|---|---|
| 0. Housekeeping | Decide branch: reconcile with `origin/two-brain` first (recommended) so rules land on one tracker. Generate `he/wake_pte_he.c` and `he/test_wake.h` via `model/wake/export_wake.py` (heavy: run on a capable machine, not this laptop). Fix host `cassert` toolchain | HE image does not build until the two files exist; no on-board wake test before that |
| 1. Measure | Board logging (C4, V7, frame age, score vs servo motion). Record the YOLO calibration set | feeds Phase 2/3 numbers |
| 2. Tracker | Implement A, B, C1, C3, D2, D3 with scenario tests, one target at a time | tests per rule above |
| 3. Voice | V1, V3, V4, V5 and the people-only switch/no semantics; extend `test_brain` and `test_orchestrator` | G6 needs a `view` change |
| 4. Trials | Real-voice and real-person sessions per Validation | exploratory results, honestly reported |
| 5. Demo | Scripted run, fallbacks (V1, D3), a one-page operator card of the words | |

## Decisions needed from you

1. **After a person is lost (D2):** hold the pose forever until a voice command (current), or hold 8 s then patrol (recommended: a turret staring at an empty spot reads as broken in a demo). This reverses part of the earlier hold-target design.
2. **Where to implement:** merge `origin/two-brain` first (recommended; it already ported part of our tracker and wake gating, and conflicts will grow), or build on our branch and port by hand.
3. **Wake word in the demo:** required (recommended, V1) or off.
4. **Real voices:** how many speakers and what rooms are available for Phase 4.
