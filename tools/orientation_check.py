#!/usr/bin/env python3
"""Check the camera orientation and the steering direction with only the ESP32 camera head.

    .venv/bin/python tools/orientation_check.py [--rotate 0|90|180|270]

Reads the head's live frames over its USB console (preview mode), turns them like the E8
does (WALDO_CAM_ROTATE), finds faces with the same BlazeFace model (float, on the host)
and applies the tracker's steering maths (src/waldo/tracker.cpp). The window shows the
frame the tracker sees and an arrow for where WALDO would turn.

Hold the head the way it is mounted on the turret and move around: the arrow must point
toward you. Better test, the way the servos do it: turn the head the way the text says
and the face must move to the centre cross. Keys: r = next rotation, h / v = toggle the
sensor's mirror / flip (on the ESP32), q = quit.
"""
import argparse
import glob
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import serial
import torch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "model"))
from model import WaldoFace  # noqa: E402

HDR = struct.Struct("<4sHHBBHI")
ANCHORS = np.load(ROOT / "model" / "blazeface" / "anchors.npy")[:, :2]
HFOV, DEADBAND = 60.0, 1.5          # tracker.cpp (degrees across the long side)
SCORE, NMS_IOU = 0.75, 0.3          # face_detector.cpp


LINK = None


def frames(port):
    """Yields (format, w, h, payload) from the head's preview stream."""
    global LINK
    s = LINK = serial.Serial(port, 921600, timeout=1)
    s.reset_input_buffer()
    s.write(b"p")   # preview on: every frame also goes to USB
    buf = b""
    try:
        while True:
            buf += s.read(s.in_waiting or 1)
            i = buf.find(b"WLDF")
            if i < 0:
                buf = buf[-3:]
                continue
            if len(buf) < i + HDR.size:
                continue
            _, w, h, fmt, _, _, n = HDR.unpack(buf[i:i + HDR.size])
            end = i + HDR.size + n + 2
            if n > 65536:
                buf = buf[i + 4:]
                continue
            while len(buf) < end:
                buf += s.read(end - len(buf))
            yield fmt, w, h, buf[i + HDR.size:i + HDR.size + n]
            buf = buf[end:]
    finally:
        s.write(b"p")   # preview off again


def to_gray(fmt, w, h, data):
    """The tracker's frame: what camlink_rx.cpp produces (160x120 gray)."""
    if fmt == 2:
        rgb = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        g = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        return cv2.resize(g, (g.shape[1] // 2, g.shape[0] // 2), interpolation=cv2.INTER_AREA)
    return np.frombuffer(data, np.uint8).reshape(h, w)


def rotate(img, deg):
    return {0: img, 90: cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE), 180: cv2.rotate(img, cv2.ROTATE_180),
            270: cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)}[deg]


def faces(net, img):
    """BlazeFace as on the board: letterbox to 128 (pad = 0.0), decode anchors, NMS."""
    h, w = img.shape
    s = 128 / max(w, h)
    sw, sh = round(w * s), round(h * s)
    ox, oy = (128 - sw) // 2, (128 - sh) // 2
    x = np.zeros((128, 128), np.float32)
    x[oy:oy + sh, ox:ox + sw] = cv2.resize(img, (sw, sh), interpolation=cv2.INTER_NEAREST) / 127.5 - 1
    with torch.no_grad():
        out = net(torch.from_numpy(x).reshape(1, 128, 128, 1))[0].numpy()
    score = 1 / (1 + np.exp(-np.clip(out[:, 0], -100, 100)))
    keep = np.where(score >= SCORE)[0]
    boxes = []
    for i in keep[np.argsort(-score[keep])]:
        cx, cy = out[i, 1] / 128 + ANCHORS[i, 0], out[i, 2] / 128 + ANCHORS[i, 1]
        bw, bh = out[i, 3] / 128, out[i, 4] / 128
        b = np.array([(cx - bw / 2) * 128 - ox, (cy - bh / 2) * 128 - oy, bw * 128, bh * 128]) / s
        if all(iou(b, k) < NMS_IOU for k, _ in boxes):
            boxes.append((b, score[i]))
    return boxes


def iou(a, b):
    ix = max(0, min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1]))
    return ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy + 1e-6)


def steer(box, w, h):
    """tracker.cpp: +pan turns the turret left, +tilt points up."""
    cx, cy = box[0] + box[2] / 2, box[1] + box[3] / 2
    deg_per_px = HFOV / max(w, h)   # the field of view goes with the sensor's long side
    err_pan = -(cx - w / 2) * deg_per_px
    err_tilt = -(cy - h / 2) * deg_per_px
    pan = "LEFT" if err_pan > DEADBAND else "RIGHT" if err_pan < -DEADBAND else ""
    tilt = "UP" if err_tilt > DEADBAND else "DOWN" if err_tilt < -DEADBAND else ""
    return err_pan, err_tilt, " + ".join(p for p in (pan, tilt) if p) or "centred"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270))
    args = ap.parse_args()
    port = (glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*"))[0]
    net = WaldoFace().eval()
    rot = args.rotate
    print(f"camera head on {port}; rotation {rot}. Keys: r = rotation, h = mirror, v = flip, q = quit")
    t0, n = time.time(), 0
    for fmt, w, h, data in frames(port):
        img = rotate(to_gray(fmt, w, h, data), rot)
        fh, fw = img.shape
        view = cv2.cvtColor(cv2.resize(img, (fw * 4, fh * 4), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
        found = faces(net, img)
        cv2.drawMarker(view, (fw * 2, fh * 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        text = "no face"
        if found:
            b, sc = found[0]
            ep, et, text = steer(b, fw, fh)
            p0 = (int(b[0] * 4), int(b[1] * 4))
            cv2.rectangle(view, p0, (int((b[0] + b[2]) * 4), int((b[1] + b[3]) * 4)), (0, 255, 0), 2)
            # Arrow: where the turret would turn, drawn in the frame (left = left, up = up).
            c = (fw * 2, fh * 2)
            cv2.arrowedLine(view, c, (int(c[0] - ep * 6), int(c[1] - et * 6)), (0, 200, 255), 3, tipLength=0.25)
            text = f"WALDO turns {text}   (pan {ep:+.1f}, tilt {et:+.1f} deg, score {sc:.2f})"
        n += 1
        cv2.putText(view, f"rotate {rot}  {fw}x{fh}  {n / (time.time() - t0):.1f} fps", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(view, text, (8, fh * 4 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.imshow("WALDO orientation check", view)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("r"):
            rot = (rot + 90) % 360
            print(f"rotation {rot}")
        if k in (ord("h"), ord("v")):
            LINK.write(bytes([k]))
            print("toggled " + ("mirror" if k == ord("h") else "flip") + " on the camera head")


if __name__ == "__main__":
    main()
