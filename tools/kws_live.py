#!/usr/bin/env python3
"""Try WALDO's voice commands on a laptop microphone, with the board's listening logic.

    <python with ai_edge_litert, sounddevice, numpy> tools/kws_live.py            # live
    <python ...> tools/kws_live.py --test [--reps 5] [--out kws_test]            # scripted test

Same pipeline as src/kws.cpp: 16 kHz audio, an MFCC frame every 20 ms (model/kws/mfcc.py,
the maths of src/waldo/kws_mfcc.c), MicroNet-M every 200 ms on the last 1 s, the mean of
the last 3 classifications, a word when that mean reaches the threshold (0.70), the same
word again only after 1 s. The model is Arm's int8 TFLite original, which our int8
ExecuTorch export matches on the sample clips.

--test asks for each command word several times, then for some ordinary talking and some
quiet, saves the audio, and prints hits and false triggers for a range of thresholds, so
the threshold in src/kws.cpp can be picked from data. The laptop's microphone is not the
DevKit's PDM pair: treat the numbers as a check of the words and the logic, then repeat the
counts on the board.
"""
import argparse
import functools
import sys
import time
import wave
from pathlib import Path

import numpy as np

print = functools.partial(print, flush=True)  # noqa: A001  (live output through pipes)
import sounddevice as sd
from ai_edge_litert.interpreter import Interpreter

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "model" / "kws"))
import mfcc as M  # noqa: E402

LABELS = (ROOT / "model" / "kws" / "micronet_kws_labels.txt").read_text().split()
WORDS = 10                 # labels before _silence_ / _unknown_
CLASSIFY_EVERY = 10        # MFCC frames (20 ms) between classifications
SMOOTH = 3
THRESHOLD = 0.70
REPEAT_MS = 1000
COMMANDS = ["off", "on", "go", "stop", "left", "right", "up", "down", "no", "yes"]


class Model:
    def __init__(self):
        self.it = Interpreter(str(ROOT / "model" / "kws" / "kws_micronet_m.tflite"))
        self.it.allocate_tensors()
        self.i = self.it.get_input_details()[0]
        self.o = self.it.get_output_details()[0]

    def probs(self, feats):
        s, z = self.i["quantization"]
        q = np.clip(np.round(feats / s + z), -128, 127).astype(np.int8)
        self.it.set_tensor(self.i["index"], q.reshape(self.i["shape"]))
        self.it.invoke()
        s, z = self.o["quantization"]
        logits = (self.it.get_tensor(self.o["index"]).astype(np.float32).ravel() - z) * s
        e = np.exp(logits - logits.max())
        return e / e.sum()


def listen_offline(model, audio, threshold):
    heard, hist, last, last_t = [], [], -1, -1e9
    step = CLASSIFY_EVERY * M.STRIDE
    for end in range(M.WINDOW, len(audio) + 1, step):
        t = end / M.RATE
        hist = ([model.probs(M.mfcc(audio[end - M.WINDOW:end]))] + hist)[:SMOOTH]
        avg = np.mean(hist, axis=0)
        best = int(np.argmax(avg))
        if best < WORDS and avg[best] >= threshold and (best != last or (t - last_t) * 1000 > REPEAT_MS):
            heard.append((t, LABELS[best], float(avg[best])))
            last, last_t, hist = best, t, []
    return heard


def record(seconds):
    a = sd.rec(int(seconds * M.RATE), samplerate=M.RATE, channels=1, dtype="int16")
    sd.wait()
    return a.ravel()


def save_wav(path, audio):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(M.RATE)
        w.writeframes(audio.tobytes())


def live(model):
    print(f"listening on '{sd.query_devices(kind='input')['name']}': say off, on, go, stop, left, right, "
          f"up, down, no, yes (ctrl-C to quit)")
    buf = np.zeros(M.WINDOW, np.int16)
    hist, last, last_t, hops = [], -1, -1e9, 0
    with sd.InputStream(samplerate=M.RATE, channels=1, dtype="int16", blocksize=M.STRIDE) as s:
        while True:
            block, _ = s.read(M.STRIDE)
            buf = np.concatenate([buf[M.STRIDE:], block.ravel()])
            hops += 1
            if hops < M.WINDOW // M.STRIDE or hops % CLASSIFY_EVERY:
                continue
            p = model.probs(M.mfcc(buf))
            hist = ([p] + hist)[:SMOOTH]
            avg = np.mean(hist, axis=0)
            best, now = int(np.argmax(avg)), time.time()
            rms = float(np.sqrt(np.mean(buf[-3200:].astype(np.float32) ** 2)))
            if best < WORDS and avg[best] >= THRESHOLD and (best != last or (now - last_t) * 1000 > REPEAT_MS):
                print(f"HEARD: {LABELS[best]:6s} ({avg[best]:.2f})   mic rms {rms:.0f}")
                last, last_t, hist = best, now, []
            elif int(np.argmax(p)) < WORDS and p.max() > 0.5:
                print(f"  maybe {LABELS[int(np.argmax(p))]} ({p.max():.2f}, avg {avg[int(np.argmax(p))]:.2f})")


def test(model, reps, out):
    out.mkdir(parents=True, exist_ok=True)
    takes = []
    print(f"{len(COMMANDS)} words x {reps}. Say each word once, at the distance you'll stand from WALDO,\n"
          f"when it says GO. Each take records 2.5 s.\n")
    input("press enter to start")
    for word in COMMANDS:
        for r in range(reps):
            print(f"  [{word}] {r + 1}/{reps} ... GO", end="", flush=True)
            a = record(2.5)
            save_wav(out / f"{word}_{r}.wav", a)
            takes.append((word, a))
            print()
            time.sleep(0.3)
    print("\nNow 20 s of ordinary talking (anything but the command words) for false triggers ... GO")
    talk = record(20)
    save_wav(out / "talk.wav", talk)
    print("Now 20 s of quiet room ... GO")
    quiet = record(20)
    save_wav(out / "quiet.wav", quiet)
    report(model, takes, talk, quiet)


def report(model, takes, talk, quiet):
    print(f"\n{'threshold':>9} {'hits':>6} {'wrong word':>10} {'missed':>6} {'false (talk)':>12} {'false (quiet)':>13}")
    per_word = {}
    for th in (0.5, 0.6, 0.7, 0.8, 0.9):
        hit = wrong = miss = 0
        for word, a in takes:
            # pad with silence so a word at the end still gets three classifications
            heard = listen_offline(model, np.concatenate([np.zeros(M.WINDOW // 2, np.int16), a,
                                                          np.zeros(M.WINDOW // 2, np.int16)]), th)
            got = [w for _, w, _ in heard]
            if word in got:
                hit += 1
            elif got:
                wrong += 1
            else:
                miss += 1
            if th == THRESHOLD:
                per_word.setdefault(word, []).append(got[0] if got else "-")
        ft = len(listen_offline(model, talk, th))
        fq = len(listen_offline(model, quiet, th))
        print(f"{th:>9.2f} {hit:>6} {wrong:>10} {miss:>6} {ft:>12} {fq:>13}")
    print(f"\nat {THRESHOLD} (src/kws.cpp), what each take was heard as:")
    for word, got in per_word.items():
        print(f"  {word:6s} {' '.join(got)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--out", type=Path, default=Path("kws_test"))
    ap.add_argument("--report", action="store_true", help="re-score a saved --out directory")
    args = ap.parse_args()
    model = Model()
    if args.report:
        rd = lambda p: np.frombuffer(wave.open(str(p)).readframes(10 ** 8), np.int16)
        takes = [(p.stem.rsplit("_", 1)[0], rd(p)) for p in sorted(args.out.glob("*_*.wav"))]
        report(model, takes, rd(args.out / "talk.wav"), rd(args.out / "quiet.wav"))
    elif args.test:
        test(model, args.reps, args.out)
    else:
        try:
            live(model)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
