#!/usr/bin/env python3
"""Record real human samples from the Mac microphone for train/eval (never faked).

    python model/wake/record_real.py --speaker anna --n 20

Prompts the speaker for N takes of "Hey Waldo" (pos_NN.wav) and N takes of other phrases
(neg_NN.wav: ordinary sentences and confusable phrases from config.yaml) into
<paths.real>/<speaker>/ as 16 kHz mono int16 wavs of 2.5 s each. Each speaker directory is a
unit of the split: list its name under real.train_speakers / val_speakers in config.yaml to
train on it; every other speaker is TEST-only. Use a pseudonym for --speaker: these are voice
recordings (personal data); keep them out of the repo and get the speaker's consent.
The first run compiles record_mic.swift (swiftc) and macOS asks for microphone permission.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wakelib  # noqa: E402

SECONDS = 2.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speaker", required=True, help="pseudonym, becomes the directory name")
    ap.add_argument("--n", type=int, default=20, help="takes of each class")
    args = ap.parse_args()
    cfg = wakelib.load_config()
    out = Path(cfg["paths"]["real"]) / args.speaker
    out.mkdir(parents=True, exist_ok=True)
    exe = wakelib.HERE / "record_mic"
    if not exe.exists():
        subprocess.run(["swiftc", "-O", str(wakelib.HERE / "record_mic.swift"), "-o", str(exe)], check=True)
    rng = np.random.default_rng(int(time.time()))
    others = cfg["tts"]["confusable"] + cfg["tts"]["sentences"]
    for i in range(args.n):
        for kind, phrase in (("pos", "Hey Waldo"), ("neg", others[rng.integers(len(others))])):
            f = out / f"{kind}_{i:02d}.wav"
            input(f"[{i + 1}/{args.n}] say \"{phrase}\" naturally after Enter and the beep ... ")
            subprocess.run(["afplay", "/System/Library/Sounds/Pop.aiff"], check=False)
            subprocess.run([str(exe), str(f), str(SECONDS)], check=True)
            a = wakelib.read_wav(f)
            print(f"  saved {f.name}: peak {np.abs(a).max()} of 32767" + ("  (very quiet, retake?)" if np.abs(a).max() < 500 else ""))
    print(f"{args.n} + {args.n} takes in {out}")


if __name__ == "__main__":
    main()
