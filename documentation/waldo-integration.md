# Combining this repo with the team's other repos

Written 2026-10-18 on branch `two-brain-waldo-integration` (from `two-brain` bd468ce). Labels: `host` = laptop test, `FVP`, `board` = on a DK-E8, `UNVERIFIED`.
Rule used for things that do the same job: **the newest, working addition wins**; the older one stays only as a fallback or is removed.

## What overlaps, and which wins

| Job | This repo (`two-brain`) | WALDO (vision lead, `vision/yolov8-on-board`) | WALDO-firmware (turret team) | Keep |
|---|---|---|---|---|
| Camera link: colour frames from the ESP32 head | JPEG 320x240 + TJpgDec on the HP (`src/waldo/jpeg_frame.c`), `camlink.h` identical to firmware `d77adb2` | RGB565 160x120, never applied to a head, its format id 2 **collides** with the head's JPEG id 2 | JPEG default in the head (`d77adb2`) | **JPEG, this repo.** WALDO's parser must adopt id 2 = JPEG. |
| Motion layer, servo calibration | `motion.*`, `servo_cal.h`, identical to `firmware/common` (`d77adb2`) | vendored copy of the older `a78f9c0`; identical except `camlink.h` | the origin | One source: WALDO-firmware `firmware/common`. |
| Object detector | YOLOv8n 28 classes, **int8 input and outputs** (newest commit), 3.0 MB | same 28 classes (this repo took our `classes.py`), float I/O, 3.5 MB, CPU float conversion ~31 ms/frame (`board`) | face detector on the U55 | **This repo's export** is newer; adopt it in the single image after its board test. Not yet run on a board (`FVP` only). |
| Steering | `tracker.cpp` (P control on pixel error, class follow, switch by direction) | `waldo_aim.h` (same law + person/animal laser interlock, lock mode with rank/next/prev, 24 + 21 board checks) | `waldo.cpp` P controller | **This repo's tracker for the flashlight demo**; WALDO's interlock stays the reference for any laser payload (this repo has no laser path). |
| State machine, patrol, hold | `orchestrator.cpp` (PARKED/SENTRY/TRACK/HOLD, patrol with per-stop memory) | none | none | This repo. The patrol also solves a gap found in WALDO: the camera is boresighted with the beam, so the parked pose (tilt -90) sees only the deck and nothing un-parks it. |
| Voice | HE core: MFCC + MicroNet-M on the U55, 10 words, `brain_link.h` mailbox | none (planned only) | keyword spotter planned in `WALDO-cad/PLAN.md` | This repo. |
| Standalone boot | powers SRAM0/1 through the Secure Enclave before `main()` uses them (`board/DevKit-E8/main.c`) | **no such call** in the WALDO firmware: `UNVERIFIED` whether it boots without a debugger | n/a | This repo's approach. Check WALDO's image before relying on it unplugged. |
| On-board test harness | `FVP` self-tests + `make -C tests/host` | J-Link mailbox/inject harness (`scripts/test_*_board.py`), 45 board checks | none | Both: different layers. |

Not overlapping: the printed turret (WALDO-cad), servo zeroing and the WiFi build (WALDO-firmware), the J-Link test tooling and Open Images work (WALDO).

## Changes made here (`host` tests, `make -C tests/host`)

1. **Hold target** (`orch_set_hold_target`, default on): a lost target stays the target. TRACK is not left for SENTRY, the pose is held, the chosen class survives and tracking resumes when it reappears. `orch_set_hold_target(false)` restores the original 5 s fall-back (the original tests run in that mode).
2. **Class survives SENTRY re-entry** ("stop"/"go"); only a fresh start from PARKED resets it to people.
3. **Continuity after loss:** the tracker remembers where the target was last seen, so a bigger box elsewhere does not take over when it returns.
4. **`orch_track_class(cls)`** / `tracker_set_target_class(cls)`: the HP-side entry for "track cup". **Nothing calls it yet**: words reach the orchestrator only as KWS labels, and the KWS has 10 words (on/off/go/stop/left/right/up/down/yes/no). A "track <object>" command needs either more words in the KWS or a console path. **UNVERIFIED / not done.**
5. **Closed-loop steering test** (`test_tracking_loop.cpp`): the real `tracker.cpp` + `motion.cpp` against a simulated camera. Signs are right (target left/up gives +pan/+tilt). Settle error 0.0 - 3.7 deg and pan overshoot 0.0 - 4.4 deg across a true lens FOV of 40 - 100 deg (the tracker assumes 60), at 5 and 10 fps with 100 ms latency; a 10 deg/s walker stays in view (RMS lag 0.4 - 2.2 deg). The FOV scales the deadband and the gain: **measure the real FOV**. Host simulation only.

Mutation checks: reverting hold target, continuity or the class survival each makes a test fail.

## Open

- Nothing here has run on a board. The two cores, YOLO int8 and the JPEG path are `FVP`/`host` only (their DevKit failed).
- WALDO still needs: JPEG id 2 in its parser, `vendor/` refreshed to `d77adb2`, and the SRAM power-up checked.
- "Hey Waldo": no wake word exists (README next steps); today every heard word acts.
- The person/animal interlock is not in this repo's tracker; do not add a laser payload here without it.
