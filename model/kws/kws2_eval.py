#!/usr/bin/env python3
"""Gate report: KWS v2 vs the current MicroNet-M with the board's listening logic
(src/kws.cpp: a 1 s window every 200 ms, the mean of 3, threshold, 1 s repeat guard).

    python -I model/kws/kws2_eval.py K --weights K/runs/r1/best.npz [--th 0.8] [--int8 MLOPS.yml]

Held-out data only: Speech Commands' test speakers, the ElevenLabs and say test voices,
the test part of the background noise.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kws2_data as D  # noqa: E402
import kws2_train as T  # noqa: E402
import kws_vocab as V  # noqa: E402
from micronet import MicroNetM  # noqa: E402

CONDS = {"clean": {}, "snr20": {"snr": 20}, "snr10": {"snr": 10}, "board": {"board": True},
         "room+board": {"room": True, "board": True}}


class Runner:
    def __init__(self, model, labels, dev):
        self.m, self.labels, self.dev = model.eval().to(dev), labels, dev
        self.words = len(labels) - 2

    def probs(self, X):
        out = []
        with torch.no_grad():
            for i in range(0, len(X), 4096):
                x = torch.from_numpy(X[i:i + 4096]).unsqueeze(-1).to(self.dev)
                out.append(torch.softmax(self.m(x), 1).cpu().numpy())
        return np.concatenate(out) if out else np.zeros((0, len(self.labels)))


def windows_of(audio):
    f = D.mfcc_stream(audio)
    starts = list(range(0, len(f) - 49 + 1, 10))
    return np.stack([f[s:s + 49] for s in starts]) if starts else np.zeros((0, 49, 10), np.float32)


def listen(p, words, th, labels, hop_s=0.2):
    heard, hist, last, last_t = [], [], -1, -9.0
    for i, pr in enumerate(p):
        hist = [pr] + hist[:2]
        avg = np.mean(hist, 0)
        b = int(avg.argmax())
        t = i * hop_s
        if b < words and avg[b] >= th and (b != last or t - last_t > 1.0):
            heard.append(labels[b])
            last, last_t, hist = b, t, []
    return heard


def run_clips(runners, clips, cond, bank, th, seed=0):
    """clips: list of (audio, target label, tag). -> {runner: list of (target, heard list, tag)}"""
    rng = np.random.default_rng(seed)
    wins = []
    for a, tgt, tag in clips:
        x = np.concatenate([np.zeros(8000, np.float32), a, np.zeros(8000, np.float32)])
        x = D.degrade(x, np.random.default_rng(rng.integers(1 << 31)), bank, cond=cond) if cond else x
        w = windows_of(x)
        wins.append(w)
    X = np.concatenate(wins).astype(np.float32)
    out = {}
    for name, r in runners.items():
        P = r.probs(X)
        res, o = [], 0
        for (a, tgt, tag), w in zip(clips, wins):
            res.append((tgt, listen(P[o:o + len(w)], r.words, th, r.labels), tag))
            o += len(w)
        out[name] = res
    return out


def score(res, words):
    per = {}
    wrong = 0
    for tgt, heard, _ in res:
        if tgt not in words:
            continue
        per.setdefault(tgt, []).append(tgt in heard)
        wrong += (tgt not in heard) and bool(heard)
    n = sum(len(v) for v in per.values())
    return {w: 100 * np.mean(v) for w, v in per.items()}, 100 * sum(sum(v) for v in per.values()) / max(n, 1), wrong


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("k", type=Path)
    ap.add_argument("--weights", type=Path, required=True)
    ap.add_argument("--th", type=float, default=0.80)
    ap.add_argument("--device", default="mps" if torch.backends.mps.is_available() else "cpu")
    ap.add_argument("--sc-per-word", type=int, default=250)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    dev = torch.device(a.device)
    old_labels = ["down", "go", "left", "no", "off", "on", "right", "stop", "up", "yes", "_silence_", "_unknown_"]
    runners = {"old": Runner(MicroNetM(), old_labels, dev), "new": Runner(T.build(a.weights), V.LABELS, dev)}
    corpus = D.load_corpus(a.k)["test"]
    sc = D.sc_index(a.k)["test"]
    bank = D.noise_bank(a.k)["test"]
    rng = np.random.default_rng(5)
    report = {}

    # A: Speech Commands test speakers (real people), the 10 old words + dog/cat
    sc_clips = []
    for w in V.SC_LABELS:
        items = [p for p, ww, _ in sc if ww == w]
        for i in rng.choice(len(items), min(a.sc_per_word, len(items)), replace=False):
            sc_clips.append((D.read_wav(items[i]), w, "sc"))
    # B: TTS test voices
    tts_clips = [(x, l, src) for x, l, kind, spk, src in corpus if kind == "word"]
    print(f"test: {len(sc_clips)} SC clips, {len(tts_clips)} TTS word clips "
          f"({len({c[3] for c in corpus})} voices)", flush=True)
    for setname, clips, conds in (("SC", sc_clips, ("clean", "snr10", "board")),
                                  ("TTS", tts_clips, ("clean", "snr20", "snr10", "board", "room+board"))):
        for c in conds:
            res = run_clips(runners, clips, CONDS[c], bank, a.th)
            so = score(res["old"], V.COMMANDS)
            sn = score(res["new"], V.COMMANDS)
            sx = score(res["new"], V.LABELS[:V.WORDS])
            report[f"{setname}/{c}"] = {"old_cmd": so, "new_cmd": sn, "new_all": sx}
            print(f"\n{setname} {c}: 10 old words hit  old {so[1]:.1f}%  new {sn[1]:.1f}%   |  new, all {V.WORDS} words {sx[1]:.1f}% (wrong {sx[2]})")
            print("   " + "  ".join(f"{w}:{so[0].get(w, float('nan')):.0f}/{sx[0].get(w, float('nan')):.0f}" for w in V.LABELS[:V.WORDS]))
            if setname == "TTS":
                conf = {}
                for tgt, heard, _ in res["new"]:
                    for h in heard:
                        if h != tgt:
                            conf[(tgt, h)] = conf.get((tgt, h), 0) + 1
                top = sorted(conf.items(), key=lambda kv: -kv[1])[:8]
                print("   confusions (said->heard):", ", ".join(f"{t}->{h}:{n}" for (t, h), n in top))
                n_cup = sum(1 for t, _, _ in res["new"] if t == "cup")
                n_up = sum(1 for t, _, _ in res["new"] if t == "up")
                report[f"{setname}/{c}"]["cup_up"] = (100 * conf.get(("cup", "up"), 0) / max(n_cup, 1),
                                                      100 * conf.get(("up", "cup"), 0) / max(n_up, 1))

    # C: false triggers on held-out talk: TTS sentences + confusables + SC test unknown words + noise
    talk = [x for x, l, kind, spk, src in corpus if kind in ("sentence", "conf")]
    unk = [p for p, w, _ in sc if w not in V.SC_LABELS]
    for i in rng.choice(len(unk), 600, replace=False):
        talk.append(D.read_wav(unk[i]))
    order = rng.permutation(len(talk))
    stream = np.concatenate([np.concatenate([talk[i], np.zeros(int(rng.uniform(0.2, 1.0) * 16000), np.float32)]) for i in order])
    minutes = len(stream) / 16000 / 60
    for c in ("clean", "board"):
        x = D.degrade(stream.copy(), np.random.default_rng(11), bank, cond=CONDS[c]) if CONDS[c] else stream
        # long stream: chunk the MFCC stream so memory stays small
        heard = {}
        for name, r in runners.items():
            P = r.probs(windows_of(x))
            heard[name] = listen(P, r.words, a.th, r.labels)
        fo = len(heard["old"]) / minutes
        fn = len(heard["new"]) / minutes
        fn_cmd = sum(h in V.COMMANDS for h in heard["new"]) / minutes
        report[f"FA/{c}"] = {"old": fo, "new": fn, "new_cmd": fn_cmd, "minutes": minutes}
        cnt = {}
        for h in heard["new"]:
            cnt[h] = cnt.get(h, 0) + 1
        print(f"\nfalse triggers, {minutes:.1f} min held-out talk, {c}: old {fo:.2f}/min  new {fn:.2f}/min "
              f"(old words only {fn_cmd:.2f}/min); new by word {cnt}")
    if a.json:
        a.json.write_text(json.dumps(report, default=lambda o: o if not isinstance(o, np.floating) else float(o), indent=1))


if __name__ == "__main__":
    main()
