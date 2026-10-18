#!/usr/bin/env python3
"""Augmented 1 s training/validation windows -> MFCCs (K/feats/{train,val}.npz).

    python -I model/kws/kws2_featurize.py K [--copies 8] [--workers 10]
"""
import argparse
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kws2_data as D  # noqa: E402
import kws_vocab as V  # noqa: E402

G = {}


def build_jobs(split, copies, rng):
    corp, sc = G["corpus"][split], G["sc"][split]
    jobs = []   # (type, ref, label, seed)
    unk_sc = [i for i, (_, w, _) in enumerate(sc) if w not in V.SC_LABELS]
    lab_sc = [i for i, (_, w, _) in enumerate(sc) if w in V.SC_LABELS]
    train = split == "train"
    for i in lab_sc:
        w = sc[i][1]
        for _ in range((4 if w in ("dog", "cat") else 3) if train else 1):
            jobs.append(("sc", i, D.LABEL_ID[w]))
    for i in rng.choice(unk_sc, min(len(unk_sc), 16000 if train else 3000), replace=False):
        jobs.append(("sc", int(i), D.UNK))
    for i, (a, label, kind, spk, src) in enumerate(corp):
        if kind == "word":
            c = (copies if src == "el" else max(2, copies - 2)) if train else 2
            jobs += [("tts", i, D.LABEL_ID[label])] * c
            if label not in V.MULTI:
                jobs += [("partial", i, D.UNK)]
        elif kind == "conf":
            jobs += [("tts", i, D.UNK)] * (3 if train else 1)
        else:
            jobs += [("crop", i, D.UNK)] * (6 if train else 3)
    jobs += [("sil", 0, D.SIL)] * (8000 if train else 1500)
    seeds = rng.integers(0, 2 ** 31, len(jobs))
    return [(t, r, l, int(s)) for (t, r, l), s in zip(jobs, seeds)]


def work(args):
    split, chunk = args
    corp, sc, bank, bab = G["corpus"][split], G["sc"][split], G["bank"][split], G["babble"][split]
    train = split == "train"
    xs, ys = [], []
    for t, r, label, seed in chunk:
        rng = np.random.default_rng(seed)
        if t == "sc":
            a = D.read_wav(sc[r][0])[:D.W]
            a = np.pad(a, (0, D.W - len(a)))
            a = np.roll(a, int(rng.integers(-1600, 1600)))
            x = D.degrade(a, rng, bank, bab)
        elif t == "tts":
            a = corp[r][0]
            if train:
                a = D.resample(a, rng.uniform(0.9, 1.12))
            x = D.degrade(D.place(a, rng), rng, bank, bab)
        elif t == "partial":
            a = corp[r][0]
            f = rng.uniform(0.2, 0.45)
            a = a[:int(len(a) * f)] if rng.random() < 0.6 else a[int(len(a) * (1 - f)):]
            x = D.degrade(D.place(a, rng), rng, bank, bab)
        elif t == "crop":
            a = corp[r][0]
            if len(a) < D.W:
                a = np.pad(a, (0, D.W - len(a)))
            s = int(rng.integers(0, len(a) - D.W + 1))
            x = D.degrade(a[s:s + D.W].copy(), rng, bank, bab)
        else:
            x = D.silence_window(bank, rng)
        xs.append(np.clip(x, -32768, 32767))
        ys.append(label)
    return D.mfcc_batch(np.stack(xs)).astype(np.float16), np.array(ys, np.int8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("k", type=Path)
    ap.add_argument("--copies", type=int, default=8)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--splits", default="train,val")
    ap.add_argument("--out", default="feats")
    a = ap.parse_args()
    t0 = time.time()
    G["corpus"] = D.load_corpus(a.k, skip_texts=V.CONTAINS_LABEL)
    G["sc"] = D.sc_index(a.k)
    G["bank"] = D.noise_bank(a.k)
    G["babble"] = {}
    for s in ("train", "val", "test"):
        unk = [p for p, w, _ in G["sc"][s] if w not in V.SC_LABELS][:300]
        G["babble"][s] = [D.read_wav(p) for p in unk]
    print({s: (len(G["corpus"][s]), len(G["sc"][s])) for s in G["sc"]}, f"loaded in {time.time() - t0:.0f} s", flush=True)
    out = a.k / a.out
    out.mkdir(exist_ok=True)
    ctx = mp.get_context("fork")
    for split in a.splits.split(","):
        jobs = build_jobs(split, a.copies, np.random.default_rng(1 if split == "train" else 2))
        chunks = [(split, jobs[i:i + 512]) for i in range(0, len(jobs), 512)]
        with ctx.Pool(a.workers) as p:
            res = p.map(work, chunks)
        X = np.concatenate([r[0] for r in res])
        y = np.concatenate([r[1] for r in res])
        np.savez(out / f"{split}.npz", X=X, y=y)
        print(split, X.shape, np.bincount(y, minlength=len(V.LABELS)).tolist(), f"{time.time() - t0:.0f} s", flush=True)


if __name__ == "__main__":
    main()
