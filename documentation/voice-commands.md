# Voice commands: every keyword and what it does

Labels: `board` = measured on the DK-E8, `host` = laptop test, `UNVERIFIED` = not tested. Code: the keyword spotter `src/kws.cpp` (decides a word is heard),
the state machine `src/waldo/orchestrator.cpp` (decides what a word does).

## How a word is heard

The M55-HE listens all the time (two on-board PDM microphones, 16 kHz). Every 20 ms it adds one MFCC frame; every 200 ms the last 1 s (49 frames x 10 coefficients) goes through
MicroNet-M on the Ethos-U55-128, which scores 12 labels. A word counts as heard when:

1. the score, **averaged over the last 3 classifications** (0.6 s), is **at least 0.80**, and
2. it is one of the 10 command words (`_silence_` and `_unknown_` never act), and
3. it is not the same word as the previous one within 1 s.

The heard word goes to the M55-HP through the mailbox in SRAM0 (`src/waldo/brain_link.h`); the HP's orchestrator acts on it.
Measured: the self-test "yes" scores 0.997 on the HE (`board`); 96 % of 260 synthetic-voice commands recognised clean and 80 % at 20 dB noise (`host`, the team's TTS test, firmware logic).
Real-voice accuracy on the board in a room is **UNVERIFIED**.

## The 10 words

States: **PARKED** head down, vision off. **SENTRY** waiting and patrolling for a target. **TRACK** following a target. **HOLD** frozen but still looking.
"Turret's left" is the image's left; +pan turns left, +tilt points up.

| Word | Meaning | PARKED | SENTRY | TRACK | HOLD |
|---|---|---|---|---|---|
| **off** | Park: head down (pan 0, tilt -90), light into the deck, vision off. Always works, also without the wake word. | no effect | parks | parks | parks |
| **on** | Wake up. | wakes to SENTRY | no effect | no effect | resumes what it was doing (see **go**) |
| **go** | Same as **on**: wake up, or resume after **stop**. | wakes to SENTRY | no effect | no effect | back to TRACK on the same target if it was tracking, else SENTRY |
| **stop** | Freeze where it is; it keeps looking (the screen and detector stay live). Always works, also without the wake word. | no effect | HOLD | HOLD | no effect |
| **left** | Move the turret toward the left. | ignored | nudge 10 deg left | **switch target** to the nearest other person to the left | nudge 10 deg left |
| **right** | Same, toward the right. | ignored | nudge 10 deg right | switch target to the nearest other person to the right | nudge 10 deg right |
| **up** | Same, upward. | ignored | nudge 10 deg up | switch target to the nearest other person above | nudge 10 deg up |
| **down** | Same, downward. | ignored | nudge 10 deg down | switch target to the nearest other person below | nudge 10 deg down |
| **no** | Drop the current target; back to following people. | no effect | back to people | back to people | back to people |
| **yes** | Heard and shown on the console, **no action** (reserved). | - | - | - | - |
| `_silence_`, `_unknown_` | Background and other words: never acted on. | | | | |

Switching target while tracking: "right" moves to the nearest other person on the right and follows them until you say another direction or **no**. Only the person class reaches the tracker (`face_detect_adapter.cpp`), so objects are never targets.
If nothing is in that direction it says so on the console and keeps its target.

## What is kept until you say something else (hold target)

By default (`orch_set_hold_target(true)`) a target that is lost stays the target: TRACK is not left for SENTRY, the turret holds its pose and keeps waiting for that kind of thing,
until you pick another with **left/right/up/down**, drop it with **no**, or freeze/park it with **stop**/**off**. Only **off** then **on** starts again from people.
(`orch_set_hold_target(false)` gives back the original behaviour: lost for 5 s -> SENTRY and back to people.)

## Wake word

When the HE reports that its wake detector is running (`brain_mailbox.wake_enabled`), the HP requires the wake phrase **"Hey Waldo"** before it acts on a command
(`orch_set_wake_required`): saying it opens a **5 s** listening window, and every accepted command renews the window, so "left ... left ... no" works in one go.
**stop** and **off** are never behind the wake word. Without a working wake detector every heard word acts, as before.
The wake detector, its data, metrics and limits are in `documentation/wake-word.md` (when present). A word is ignored outside the window and the console prints `ignored (say "Hey Waldo" first)`.

## Not possible with these words

- Following objects ("track the cup"): out of scope, the turret follows people only.
- Numbers, "faster", "light on/off": not in the vocabulary.

## Try it on the board without speaking

The HE publishes what it heard in the mailbox (`0x023FF000`): read it over J-Link, or play a command from the laptop speaker next to the board
(`say -v Samantha "stop"`) and watch `word_head` and the `HE: heard ...` log lines. The console needs SW4 on UART4 (115200 on the PRG USB port).
