#!/usr/bin/env python3
"""Fine-tune MicroNet-M (Arm's weights) to the KWS v2 vocabulary on K/feats.

    python -I model/kws/kws2_train.py K --out K/runs/r1 [--epochs 24] [--device mps]

Head 196 -> len(LABELS): the old 12 rows start from Arm's classifier (same label order),
the new rows from small random values. Phase A trains the head only, phase B everything.
Writes best.npz (micronet_m_weights.npz layout, w11/b11 resized) by validation accuracy.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kws_vocab as V  # noqa: E402
from micronet import MicroNetM  # noqa: E402

HERE = Path(__file__).resolve().parent
# The int8 input range of Arm's MicroNet-M (in_q: scale 0.2011, zero point -5). Training,
# evaluation and calibration all clip to it, so the int8 program sees what training saw.
IN_LO, IN_HI = -24.7, 26.6
OLD = ["down", "go", "left", "no", "off", "on", "right", "stop", "up", "yes", "_silence_", "_unknown_"]


def build(weights=None):
    m = MicroNetM()
    n = len(V.LABELS)
    old = m.layers[11]
    head = torch.nn.Conv2d(196, n, 1)
    with torch.no_grad():
        head.weight.normal_(0, 0.02)
        head.bias.zero_()
        for i, l in enumerate(OLD):
            head.weight[V.LABELS.index(l)] = old.weight[i]
            head.bias[V.LABELS.index(l)] = old.bias[i]
    m.layers[11] = head
    if weights is not None:
        load_npz(m, weights)
    return m


def save_npz(m, path):
    """Same layout as micronet_m_weights.npz (OHWI / 1HWC)."""
    out = dict(np.load(HERE / "micronet_m_weights.npz"))
    for i, conv in enumerate(m.layers):
        w = conv.weight.detach().cpu().numpy()
        out[f"w{i}"] = w.transpose(1, 2, 3, 0) if conv.groups > 1 else w.transpose(0, 2, 3, 1)
        out[f"b{i}"] = conv.bias.detach().cpu().numpy()
    np.savez(path, **out)


def load_npz(m, path):
    w = np.load(path)
    for i, conv in enumerate(m.layers):
        wi = w[f"w{i}"]
        conv.weight.data = torch.from_numpy((wi.transpose(3, 0, 1, 2) if conv.groups > 1 else wi.transpose(0, 3, 1, 2)).astype(np.float32).copy())
        conv.bias.data = torch.from_numpy(w[f"b{i}"].astype(np.float32).copy())


def balanced_epoch(y, rng, per_word=3500, unk=20000, sil=3000):
    idx = []
    for c in range(len(V.LABELS)):
        pool = np.flatnonzero(y == c)
        if len(pool) == 0:
            continue
        n = unk if V.LABELS[c] == "_unknown_" else sil if V.LABELS[c] == "_silence_" else per_word
        idx.append(rng.choice(pool, n, replace=len(pool) < n))
    idx = np.concatenate(idx)
    rng.shuffle(idx)
    return idx


def evaluate(m, X, y, dev, bs=4096):
    m.eval()
    preds = []
    with torch.no_grad():
        for i in range(0, len(X), bs):
            preds.append(m(X[i:i + bs].to(dev).clamp(IN_LO, IN_HI)).argmax(1).cpu())
    p = torch.cat(preds).numpy()
    per = [float((p[y == c] == c).mean()) if (y == c).any() else float("nan") for c in range(len(V.LABELS))]
    return float((p == y).mean()), per


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("k", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--epochs", type=int, default=16)
    ap.add_argument("--head-epochs", type=int, default=2)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--device", default="mps" if torch.backends.mps.is_available() else "cpu")
    ap.add_argument("--bs", type=int, default=256)
    ap.add_argument("--init", type=Path, help="start from these weights (best.npz of an earlier run)")
    ap.add_argument("--feats", default="feats")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(0)
    rng = np.random.default_rng(0)
    tr, va = np.load(a.k / a.feats / "train.npz"), np.load(a.k / a.feats / "val.npz")
    Xtr = torch.from_numpy(tr["X"].astype(np.float32)).unsqueeze(-1)
    ytr = tr["y"].astype(np.int64)
    Xva = torch.from_numpy(va["X"].astype(np.float32)).unsqueeze(-1)
    yva = va["y"].astype(np.int64)
    dev = torch.device(a.device)
    m = build(a.init).to(dev)
    acc, per = evaluate(m, Xva, yva, dev)
    print(f"start: val acc {acc:.3f}", flush=True)
    best = -1
    total = a.head_epochs + a.epochs
    for ep in range(total):
        head_only = ep < a.head_epochs
        for i, layer in enumerate(m.layers):
            for p in layer.parameters():
                p.requires_grad = (i == 11) or not head_only
        if ep == 0 or ep == a.head_epochs:
            opt = torch.optim.AdamW([p for p in m.parameters() if p.requires_grad],
                                    lr=a.lr if head_only else a.lr * 0.5, weight_decay=1e-4)
            steps = max(1, (a.head_epochs if head_only else a.epochs)) * (len(balanced_epoch(ytr, np.random.default_rng(9))) // a.bs)
            sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=opt.param_groups[0]["lr"], total_steps=steps + 10, pct_start=0.1)
        m.train()
        t0, tl, nb = time.time(), 0.0, 0
        idx = balanced_epoch(ytr, rng)
        for b in range(0, len(idx) - a.bs + 1, a.bs):
            j = idx[b:b + a.bs]
            x = Xtr[j].to(dev).clamp(IN_LO, IN_HI)
            # SpecAugment-light: time mask, small feature jitter
            if rng.random() < 0.5:
                t = int(rng.integers(0, 40))
                x[:, t:t + int(rng.integers(1, 8))] = x[:, t:t + 1].mean()
            yb = torch.from_numpy(ytr[j]).to(dev)
            loss = F.cross_entropy(m(x), yb, label_smoothing=0.05)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            tl += float(loss)
            nb += 1
        acc, per = evaluate(m, Xva, yva, dev)
        words = np.nanmean(per[:V.WORDS])
        print(f"epoch {ep + 1}/{total}{' head' if head_only else ''}: loss {tl / nb:.3f} val acc {acc:.3f} "
              f"mean word recall {words:.3f} ({time.time() - t0:.0f} s)", flush=True)
        if words > best:
            best = words
            save_npz(m.cpu(), a.out / "best.npz")
            m.to(dev)
            (a.out / "best.txt").write_text(f"epoch {ep + 1} val acc {acc:.4f} word recall {words:.4f}\n" +
                                           "\n".join(f"{l:10s} {r:.3f}" for l, r in zip(V.LABELS, per)) + "\n")
    print((a.out / "best.txt").read_text())


if __name__ == "__main__":
    main()
