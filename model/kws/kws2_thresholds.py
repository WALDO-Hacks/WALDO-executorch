#!/usr/bin/env python3
"""Per-word thresholds for KWS v2, chosen on the VALIDATION split (never on test):
for each word the lowest threshold whose false triggers stay within budget.

    python -I model/kws/kws2_thresholds.py K --weights K/runs/r2/best.npz

Prints the --word-th string for kws2_eval.py and export_kws.py --threshold.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kws2_data as D  # noqa: E402
import kws2_eval as E  # noqa: E402
import kws2_train as T  # noqa: E402
import kws_vocab as V  # noqa: E402

GRID = [0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("k", type=Path)
    ap.add_argument("--weights", type=Path, required=True)
    ap.add_argument("--talk-budget", type=float, default=0.05, help="false triggers/min per word, ordinary talk")
    ap.add_argument("--near-budget", type=float, default=0.5, help="false triggers/min per word, near misses")
    a = ap.parse_args()
    dev = torch.device("mps" if torch.backends.mps.is_available() else "cpu")
    r = E.Runner(T.build(a.weights), V.LABELS, dev)
    corpus = D.load_corpus(a.k, skip_texts=V.CONTAINS_LABEL)["val"]
    sc = D.sc_index(a.k)["val"]
    bank = D.noise_bank(a.k)["val"]
    rng = np.random.default_rng(3)
    unk = [p for p, w, _ in sc if w not in V.SC_LABELS]
    streams = {"talk": [x for x, l, kind, *_ in corpus if kind == "sentence"] +
                       [D.read_wav(unk[i]) for i in rng.choice(len(unk), 400, replace=False)],
               "near": [x for x, l, kind, *_ in corpus if kind == "conf"]}
    fa = {s: {t: {} for t in GRID} for s in streams}
    for s, clips in streams.items():
        stream = np.concatenate([np.concatenate([c, np.zeros(int(rng.uniform(0.2, 1.0) * 16000), np.float32)]) for c in clips])
        minutes = 2 * len(stream) / 16000 / 60
        for cond in ({}, {"board": True}):
            x = D.degrade(stream.copy(), np.random.default_rng(7), bank, cond=cond) if cond else D.dither(stream)
            P = r.probs(E.windows_of(x))
            for t in GRID:
                for h in E.listen(P, r.words, t, r.labels):
                    fa[s][t][h] = fa[s][t].get(h, 0) + 1 / minutes
    # hits on validation voices/speakers, clean and board
    clips = [(c[0], c[1]) for c in corpus if c[2] == "word"]
    for w in V.SC_LABELS:
        clips += [(D.read_wav(p), w) for p in [p for p, ww, _ in sc if ww == w][:80]]
    hits = {t: {} for t in GRID}
    for cond in ({}, {"board": True}):
        wins = []
        for a_, _ in clips:
            x = np.concatenate([np.zeros(8000, np.float32), a_, np.zeros(8000, np.float32)])
            x = D.degrade(x, np.random.default_rng(int(rng.integers(1 << 31))), bank, cond=cond) if cond else D.dither(x)
            wins.append(E.windows_of(x))
        P = r.probs(np.concatenate(wins).astype(np.float32))
        for t in GRID:
            o = 0
            for (_, tgt), w in zip(clips, wins):
                hits[t].setdefault(tgt, []).append(tgt in E.listen(P[o:o + len(w)], r.words, t, r.labels))
                o += len(w)
    out = []
    for w in V.LABELS[:V.WORDS]:
        ok = [t for t in GRID if fa["talk"][t].get(w, 0) <= a.talk_budget and fa["near"][t].get(w, 0) <= a.near_budget]
        t = ok[0] if ok else GRID[-1]
        out.append(f"{w}={t}")
        print(f"{w:10s} th {t:.2f}  val hit {100 * np.mean(hits[t][w]):5.1f}%  FA/min talk {fa['talk'][t].get(w, 0):.3f} "
              f"near {fa['near'][t].get(w, 0):.2f}" + ("" if ok else "  (budget not met)"))
    print("--word-th " + ",".join(out))


if __name__ == "__main__":
    main()
