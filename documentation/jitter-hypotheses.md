# Turret jitter and aimless motion: hypotheses (UNVERIFIED)

**FOR THE PARTNER'S AGENT: read this before changing `tracker.cpp`, `orchestrator.cpp` or any threshold in them.**
Status: **HYPOTHESES from reading the code. Nothing here was measured on the board.** Do not tune a number because of this document.
Measure first (section "How to test each one"), record the log, then change one thing at a time and say which hypothesis the log supports.
Code references are to the `two-brain` tip at the time of writing (`a28137a`).

## Reported symptoms (from the user, on the board)

| # | Symptom |
|---|---|
| S1 | The camera moves around randomly sometimes, maybe when it loses someone |
| S2 | While tracking, as the person moves and the box changes, the turret jitters |
| S3 | With a napkin over the camera the turret moves around aimlessly |

## Hypotheses

### H1 (S1): a lost target triggers three different movements in a row
| Step | What the code does | Where |
|---|---|---|
| 0 to 0.7 s | Coast: keeps turning at the filter's speed, up to 25 deg in total, whenever the speed is >= 15 deg/s | `tracker.cpp` `kCoastMs` 700, `kCoastMaxDeg` 25, `kMovingDegS` 15 |
| up to 3.2 s | If last seen near the left or right edge and speed >= 15 deg/s: one 30 deg pan jump that way | `kSearchDeg` 30, `kSearchMs` 2500 |
| after 4 s | TRACK -> SENTRY: the turret goes to the nearest of 10 scan stops, then sweeps pan +-70 deg, tilt up to 20 deg | `orchestrator.cpp` `kLostToSentryMs` 4000, `kStops` |

Why it can look random: the speed estimate is noisy. In the host simulation (`test_runner`) the filter read 116 deg/s for a 40 deg/s runner, so the coast and the search can go the wrong way by up to 25 to 30 deg. A short detector dropout (under 4 s) still runs the coast and the search. The patrol afterwards is by design but looks aimless.
Prediction if true: console shows the pan moving by up to 25 or 30 deg right after `track: ... lost`, in a direction that does not match where the person went.

### H2 (S2): box noise becomes turret motion
- The aim point is the **centre of the detector box** (`tracker.cpp:246`, `st.cy = b[pick].y + b[pick].h * 0.5f`). A person's box changes shape as they move (arms, legs, turning). A box cut off by the frame edge also moves with the tilt.
- Every detector frame the turret moves **half the way** to the new aim (`kGain` 0.5, `tracker.cpp:286`), with only a **2 deg dead zone** (`deadband`). So detector noise of a few degrees is passed on to the servos.
- The camera delay is assumed **250 ms** (`kFrameAgeMs`). If the real delay differs, the tracker corrects for turning that has already happened, which gives overshoot and wobble, more with a fast person. The boot "loop delay probe" measures the real value; whether the tracker uses that figure was not checked.
- The field of view is assumed **50 deg** across the long side (`fov_long`), the lens is probably 55 to 57 deg. A wrong value changes the loop gain.
- The main loop ticks about every 100 ms on the board while a frame is processed, so the turret position history used for the delay correction is coarse.
Prediction if true: the `track:` stats line shows the error pan/tilt flipping sign between detector frames while the person stands nearly still, and the jitter grows with frame delay.

### H3 (S3): a covered camera looks like motion, then like people
- In SENTRY every frame is compared with the previous one in 8x8 cells. A cell counts if its mean change is above **18 grey levels**, and **3 cells** are enough (`kCellThresh`, `kMinCells`). Covering the lens makes the camera's auto exposure and gain shift brightness over the whole picture, which changes many cells at once.
- The orchestrator reads that as movement, steers toward the centre of the changed cells and runs the detector for 6 frames (`kInvestigateFrames`), repeatedly.
- At patrol stops the "did this stop change" check (`kBgThresh` 20, `kBgMinCells` 3) fires on a lighting change too, so YOLO runs at every stop.
- The detector can report a person in a blurry or textured blob. There is **no minimum score and no confirmation over several frames**, so the first sighting latches (rules A1 and A2 in `people-tracking-and-voice-rules.md`, not coded). The latch (`ac2fc6c`) then keeps the ghost and ignores real people until it is unmatched for 4 s.
Prediction if true: with the napkin on, the console prints `orch: motion at x,y` repeatedly, brightness in the `camera link:` line moves a lot, and possibly `track: latched onto a person` with nobody there.

## How to test each one (measure before changing anything)

| Hypothesis | Log or experiment | It is **falsified** if |
|---|---|---|
| H1 | Walk out of view at an edge, then behind a pillar; log turret pan every 100 ms plus `track:` lines | The turret does not move more than the person's own motion during the first 3 s after losing them |
| H2 | Stand still in view for 30 s; log error pan/tilt and the box centre per detector frame. Repeat while waving arms. Repeat with the delay constant changed to the probe's measured value | Box centre noise is under 1 deg and the turret still jitters, which points at the servos or the motion layer |
| H3 | Cover the camera in SENTRY for 30 s with a napkin; log brightness, `orch: motion` count, detector boxes and scores. Then cover it with a matte black cap | The motion count stays near zero with the napkin on |

Also record: frame rate, the real frame delay from the boot probe, detector scores of any ghost boxes.

## Candidate fixes (only after the log supports the matching hypothesis)

| For | Change | Check |
|---|---|---|
| H3 | Ignore a motion frame when more than about 30 % of cells change at once (exposure, not a person) | Host test: a whole-frame brightness step must not trigger motion |
| H3 | Minimum detector score and K of N confirming frames before latching (rules A1, A2) | Host test: a one-frame ghost does not latch; a steady person latches within 4 frames |
| H2 | Low-pass the aim point, widen the dead zone, or aim at a steadier anchor than the box centre | `test_runner` "standing still, noisy" must still pass; compare the sign-flip count before and after |
| H2 | Use the measured frame delay instead of the 250 ms constant | Same log as above |
| H1 | Cap the coast more tightly until the speed estimate can be trusted, or require the filter to have seen N consistent frames before coasting | `test_runner` pillar and sprint scenarios must still pass |

## Rules for whoever acts on this
1. Each change is one commit that names the hypothesis and cites the log that supports it. Say what was **not** verified.
2. Run the host tests as single targets at low priority (`nice`); do not run the whole suite on a laptop.
3. A fix that passes the host simulation is **not** confirmed on the board. State which you verified.
4. If the log contradicts a hypothesis, edit this file: mark it falsified with the evidence. Negative results stay in the record.
