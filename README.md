# WALDO: a voice-controlled turret that finds and follows, on one Alif E8

WALDO is a pan/tilt turret with a flashlight. It looks for people and objects with a
camera, points at them, and keeps following them. You steer it by voice: "stop", "go",
"left", "off". All of it runs on the **Alif Ensemble E8 DevKit**, with no PC and no cloud.
Three neural networks run through **ExecuTorch on the Ethos-U85 NPU**, and the Cortex-M55
does the rest.

Built for the Arm + Meta ExecuTorch hackathon on this repository's `hackathon` template.
The original setup guide for the template is in
[documentation/template-guide.md](documentation/template-guide.md).

<!-- demo GIF / photo goes here -->

## How it works

```mermaid
flowchart LR
    subgraph HEAD["ESP32-S3 camera head (the eye)"]
        CAM["OV3660 sensor<br/>JPEG encoder in hardware"]
    end
    subgraph E8["Alif Ensemble E8: Cortex-M55 HP + Ethos-U85"]
        RX["UART RX + JPEG decode<br/>(M55)"]
        FACE["BlazeFace<br/>faces (U85)"]
        YOLO["YOLOv8n<br/>28 object classes (U85)"]
        MOT["Motion detect<br/>(M55, no model)"]
        MIC["2 PDM mics<br/>MFCC (M55)"]
        KWS["MicroNet KWS<br/>10 words (U85)"]
        ORCH{{"Orchestrator<br/>PARKED · SENTRY · TRACK · HOLD"}}
        TRK["Tracker<br/>+ motion layer"]
        LCD["LCD: what WALDO sees"]
    end
    SERVO["Pan / tilt servos<br/>+ flashlight"]
    CAM -- "320x240 colour, 2 Mbaud UART" --> RX
    RX --> MOT & FACE & YOLO
    MIC --> KWS
    MOT & FACE & YOLO & KWS --> ORCH
    ORCH --> TRK --> SERVO
    ORCH --> LCD
```

**The hardware split.** The ESP32 is only the eye: it captures, compresses in its own
JPEG hardware, and sends. Every model and every decision runs on the E8. Each job goes to
the unit that fits it:

| Job | Runs on | Why |
|---|---|---|
| Face and object detection, keyword spotting | **Ethos-U85** via ExecuTorch | dense int8 convolutions; every operator of all three models runs on the NPU |
| MFCC audio features, JPEG decode, motion detection, box decoding and NMS | **Cortex-M55** | branchy or small work, cheaper on the CPU than a trip to the NPU |
| Microphone capture, camera bytes | PDM block, UART, both on interrupts | audio and frames arrive while the NPU runs |
| Servo PWM | UTIMER hardware | jitter-free 50 Hz, no CPU cost |

### The orchestrator

The orchestrator is one state machine
([src/waldo/orchestrator.cpp](src/waldo/orchestrator.cpp)). It decides how much work each
camera frame gets, so heavy models run only when there is something to look at.

```mermaid
stateDiagram-v2
    [*] --> SENTRY
    SENTRY --> TRACK: target found / motion
    TRACK --> SENTRY: target lost 5 s
    TRACK --> HOLD: "stop"
    SENTRY --> HOLD: "stop"
    HOLD --> SENTRY: "go"
    SENTRY --> PARKED: "off"
    TRACK --> PARKED: "off"
    HOLD --> PARKED: "off"
    PARKED --> SENTRY: "on" / "go"
```

| State | What runs | Turret |
|---|---|---|
| PARKED | microphones only | head down |
| SENTRY | motion detection on every frame (CPU only); the detector on every 4th frame | looks ahead and turns toward movement |
| TRACK | the detector on every frame | follows the target |
| HOLD | the detector on every frame | frozen |

**Voice commands:**

| Word | Effect |
|---|---|
| off | park |
| on / go | wake up or resume |
| stop | hold still |
| left / right / up / down | while tracking: **switch to the next target in that direction**; otherwise: nudge the turret 10° |
| no | drop the current target |

The screen shows the camera view with boxes on what WALDO sees (the target in green) and
a bar for the state: red PARKED, blue SENTRY, green TRACK, yellow HOLD.

## The models

All three go through the same flow as the template's model: PyTorch, then int8 PT2E with
the `EthosUQuantizer`, then the Ethos-U partitioner, then Vela, then a `.pte` embedded in
the image.

| Model | Task | Input | NPU ops | Program | Source |
|---|---|---|---|---|---|
| **BlazeFace** | face detection | 128x128 gray | 100 % | 200 KB | MediaPipe front model, first conv folded to 1 channel ([model/model.py](model/model.py)) |
| **MicroNet-M** | keyword spotting, 10 words | 49x10 MFCC (1 s at 16 kHz) | 42/42 | 170 KB | Arm ML-Zoo, rebuilt in PyTorch from its TFLite weights ([model/kws](model/kws)) |
| **YOLOv8n** | 28 object classes | 320x320 RGB | 99/99 | 3.0 MB | Ultralytics, head sliced to 28 room classes by our vision lead ([model/yolo](model/yolo)) |

Only the float-to-int8 conversions at each model's input and output stay on the CPU.

## Measured

`board` means measured on the DK-E8, and `host` means checked on a PC.

| What | Number | Where |
|---|---|---|
| BlazeFace inference | **9.4 ms** on the U85 | board |
| Full face-tracking frame: preprocess 3.6 + NPU 9.4 + decode 0.6 + display 12.1 | 25.6 ms (27 % of the CPU at 10 fps) | board |
| Boot self-test: face found on a built-in photo | `Test_result: PASS` | board |
| Microphones: both channels at 16 kHz, no FIFO errors | — | board |
| Camera link, grayscale 160x120 (v1) | 10.4 fps | board |
| Camera link, **colour JPEG 320x240** (v2) | **27.8 fps** at about 3–12 KB per frame | camera head |
| JPEG decode of a 320x240 camera frame | 0.25–0.7 ms on a laptop; the M55 is still to be measured | host |
| KWS: int8 model vs float vs Arm's TFLite original on the sample clips | agree on every word | host |
| YOLOv8n int8 on the team's 12 golden images vs the float model | agree on all but 3 near-threshold detections | host |
| YOLOv8n on a DK-E8 (the team's own export) | 29 ms per frame, mAP50 0.439 board vs 0.439 host | board |

### Optimisations

- **The camera sends JPEG**, compressed by the sensor's own hardware. Colour at 4x the
  pixels and 2.7x the frame rate, over the same 2 Mbaud UART.
- **Motion gating in SENTRY**: the detector runs on 1 frame in 4 until something moves.
- **Audio features are computed as audio arrives.** Each 20 ms of audio adds one MFCC
  frame, so a 1 s window never needs recomputing; the NPU classifies it every 200 ms.
- **The NPU only sees work it is good at.** Every operator of every model is delegated.
- **Per-stage timing**: every 2 s the board prints mean and worst time per stage
  (preprocess, NPU, decode, orchestrator, display), which is how we found that the display,
  not the NPU, is the most expensive stage.

## Hardware

| Part | Connection |
|---|---|
| Alif Ensemble E8 DevKit | PRG USB for power, flashing and the console |
| Pan servo | J10 pin 27 (P7_4, UTIMER 10) |
| Tilt servo | J10 pin 20 (P6_2, UTIMER 5) |
| Servo power | separate 6 V supply, ground shared with the E8 |
| ESP32-S3-CAM (OV3660) | GPIO14 (TX) → E8 UART0 RX (P1_4); GPIO21 (RX) ← E8 UART0 TX (P1_5); J10 pins 33 / 35 (check the TX/RX labels against your board); shared ground |

The J10 header is level-shifted to 3.3 V on the DevKit. The servo signals connect directly.

## Build and run

You need CMSIS-Toolbox 2.15, Arm GNU Toolchain 14.3 and Alif SETOOLS (see
[documentation/template-guide.md](documentation/template-guide.md)).

```sh
cp tools/env.sh.example ~/arm-tools/env.sh   # edit the paths
source ~/arm-tools/env.sh
tools/flash.sh            # build, write the boot table and the image to MRAM (SW4 on SEUART)
# SW4 to UART4, press reset; console at 115200 on the PRG USB port
```

The camera head is in the team's firmware repository (`firmware/esp32cam`, PlatformIO).

To re-export a model:

```sh
python create_ai_layer.py cmsis-executorch.cbuild-mlops.yml                   # BlazeFace -> ai_layer/
.venv/bin/python model/kws/export_kws.py cmsis-executorch.cbuild-mlops.yml    # keyword spotter -> src/kws_pte.c
<venv with ultralytics>/bin/python model/yolo/export_yolo.py cmsis-executorch.cbuild-mlops.yml <images>
make -C tests/host                                                            # orchestrator host test
```

At boot the board tests itself and prints `Test_result` (face) and `KWS_result` (a recorded
"yes"). Then it runs a servo test and starts in SENTRY.

## Things we learned about the E8

- **Booting from MRAM without a debugger leaves SRAM0 and SRAM1 unpowered.** The template's
  startup code zeroes SRAM0 before `main()`, so the image hangs silently. We power both
  banks through the Secure Enclave right after the console is up
  (`SERVICES_power_memory_req`), and startup no longer touches them
  ([board/DevKit-E8/main.c](board/DevKit-E8/main.c)).
- **The Ethos-U85 cannot reach the M55's DTCM.** ExecuTorch's pools have to sit in SRAM;
  in DTCM the NPU fails with status 0x9004.
- The UTIMER driver needs a non-NULL callback. Servo channels need compare buffering off,
  or the duty cycle never changes.
- Arm's quantizer does not fuse `relu6` with a convolution in this ExecuTorch version, and
  the partitioner then splits the graph. `clamp(0, 6)` gives the same result and keeps
  the whole network on the NPU.

## Status

**Works on the board:**
- face detection on the NPU
- camera link, servo tracking, the LCD view
- microphones
- standalone boot with the self-test

**Built and host-tested, waiting for a board:** our DevKit failed during the hackathon.
- keyword spotting live
- the orchestrator
- the JPEG camera path
- YOLOv8 in the firmware: the model is exported, but fitting it into flash still needs a
  larger app slot in MRAM

**Next steps:**
- Move always-on listening and motion detection to the **M55-HE + Ethos-U55** and put the
  HP core to sleep in SENTRY.
- Measure the power difference.

## Credits and licences

- The template: Arm (Apache-2.0).
- BlazeFace: MediaPipe weights via hollance/BlazeFace-PyTorch (Apache-2.0).
- MicroNet-M: Arm ML-Zoo (Apache-2.0).
- Speech Commands clips: CC BY 4.0.
- TJpgDec: ChaN (no restrictions on use).
- **YOLOv8: Ultralytics, AGPL-3.0.**
