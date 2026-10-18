# Audit: the int8-input/output YOLOv8n export (`model/yolo/export_yolo.py`, commit bd468ce)

Done 2026-10-18 on branch `two-brain-waldo-integration`. Labels: `host` = laptop, `board` = DK-E8, `UNVERIFIED` = not tested.
Script and logs: WALDO repo `scripts/audit_executorch_yolo.py`, `docs/logs/2026-10-18-executorch-yolo-audit-n*.log` (exploratory; fake-quant on the host is not Vela bit-exact).

## What the change is
The export strips the float<->int8 conversions from the program (ExecuTorch `QuantizeInputs/QuantizeOutputs`); the M55 quantizes pixels with a 256-entry table and reads the three int8 head maps
with the scale/zero-point written to `src/yolo/yolo_io_quant.h`. Claimed gain: the generic conversion kernels cost ~100 ms per frame on the M55.

## Verdict
**Sound and faster; two caveats.** The change moves a quantize/dequantize pair out of the graph; the numerics are the same quantization as the float-I/O export. Accuracy cost of int8 is small. The speed claim holds for the detector stages.

## Checks and results

| Check | Result |
|---|---|
| Does the committed `.pte` match the documented recipe? Recipe re-run on COCO128 (the calibration set the docstring names): input scale/zp and the three output scales/zero-points | **identical to the committed header**: in 0.003920/-128, out 0.1654/25, 0.2099/55, 0.2255/61 (`host`). The committed program was produced by this recipe on COCO128. (A different 128-image calibration gives 0.1737/26, 0.2213/56, 0.2196/63.) The calibration set is **not recorded in the repo**. |
| Input quantization | scale 0.003920 = 1/255 (0.003922), zp -128: every 8-bit pixel maps 1:1 onto int8, lossless. The firmware table reproduces `p/255 -> int8` exactly. |
| Letterbox | pad value is the quantized 114, as in the export. The firmware samples **nearest-neighbour**, the export reference is bilinear. For the real camera frames (320x240 or portrait 240x320, scale 1) they are identical; for larger images (the boot self-test photo) they differ slightly. |
| Output maps | identified by size, each with its own scale/zero-point, so the output order does not matter. Decode (`waldo_yolo_decode.h`, int8 variant) dequantizes as `(q - zp) * scale` everywhere; the class argmax on raw int8 is valid because one map shares one scale. Same maths as our float reference. |
| Accuracy, FP32 vs int8 fake-quant, 1000 seeded val2017 images (`host`) | mAP50 0.4553 vs **0.4468**: difference **-0.0086, 95% CI [-0.0165, -0.0029]** (about 2 % relative). mAP50-95 0.3227 vs 0.3170. |
| Same, our random-128 calibration (our earlier export) | 0.4519, difference to FP32 -0.0034 [-0.0102, +0.0010]. Difference between the two calibrations -0.0051 [-0.0119, +0.0027]: **not statistically resolved** at N=1000, so a re-calibration is not shown to help. |
| Same on 300 images (the set of our earlier board test) | FP32 0.4394, int8 0.4222 (-0.0172 [-0.0328, +0.0014]); our earlier board int8 on these images was 0.4389. |
| Speed on the DK-E8 (`board`, self-test frame, read over J-Link from the HP's stage counters) | letterbox + input table 5.2 ms, NPU forward 20.8 ms (NPU busy 13.6 ms, ExecuTorch overhead 7.2 ms), decode 3.7 ms: **29.7 ms per frame**. Our float-I/O pipeline was 66.5 ms per frame (`board`: conversion 30.8 + NPU 32.1 + decode 3.6). The JPEG decode is a separate stage and is not in these numbers. |

## Not verified
- **On-board accuracy** of this program: never measured (needs images fed through the board; the boot self-test shows people detected on one photo, scores 0.84/0.74/0.65 vs PC 0.84/0.70/0.65 on the FVP, per the README).
- The int8 and the float export differ by at most the calibration; board vs host fake-quant differed by +0.005 [-0.009, +0.015] for our earlier model (300 images).
- JPEG decode time and the end-to-end frame rate with the real camera head.
- Real-scene accuracy (no camera, no room-specific data): all numbers are COCO.

## Recommendations
1. Record the calibration set (file list and hash) next to `yolo_io_quant.h`; today it is only a docstring.
2. Keep the export. If accuracy matters more than ~0.9 point mAP50, re-calibrate on a larger/more varied set (include camera frames) and re-audit with N >= 1000; the host evidence does not yet show a gain.
3. Optional: bilinear letterbox when the source is not scale 1 (only the self-test photo is affected).
