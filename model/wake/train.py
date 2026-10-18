#!/usr/bin/env python3
"""Train the wake-word network (train voices + Speech Commands train split only).

    python -I model/wake/train.py --seed 1 [--width 32 --blocks 4]

Writes <paths.runs>/w{width}b{blocks}_s{seed}/model.pt. The epoch kept is the one with the best
validation TPR at 0.5 % false-positive rate per window; the TEST split is never read here.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import synth  # noqa: E402
import wakelib  # noqa: E402
from wake_model import WakeNet, n_params  # noqa: E402


def features(audio):
    return torch.from_numpy(wakelib.mfcc_batch(np.round(audio).astype(np.int16))).unsqueeze(-1)


def val_set(corpus, cfg, n_pos=1500):
    rng = np.random.default_rng(cfg["seed"] + 99)
    ratios = cfg["train"]["neg_ratio"]
    audio, y = [], []
    for _ in range(n_pos):
        audio.append(synth.positive(corpus, rng, speed=False))
    kinds = list(ratios)
    p = np.array([ratios[k] for k in kinds]) / sum(ratios.values())
    for k in rng.choice(len(kinds), 3 * n_pos, p=p):
        audio.append(synth.negative(corpus, rng, kinds[k]))
    y = np.r_[np.ones(n_pos), np.zeros(3 * n_pos)]
    return features(np.stack(audio)), y


def tpr_at_fpr(scores, y, fpr):
    thr = np.quantile(scores[y == 0], 1 - fpr)
    return float((scores[y == 1] > thr).mean())


def predict(model, x, batch=2048):
    model.eval()
    dev = next(model.parameters()).device
    with torch.no_grad():
        return np.concatenate([torch.softmax(model(x[i:i + batch].to(dev)), 1)[:, 1].cpu().numpy() for i in range(0, len(x), batch)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--width", type=int)
    ap.add_argument("--blocks", type=int)
    ap.add_argument("--epochs", type=int)
    args = ap.parse_args()
    cfg = wakelib.load_config()
    t = cfg["train"]
    width, blocks, epochs = args.width or t["width"], args.blocks or t["blocks"], args.epochs or t["epochs"]
    torch.manual_seed(args.seed)
    dev = torch.device("mps" if torch.backends.mps.is_available() else "cpu")   # ~30x faster than the CPU here; not bit-reproducible
    rng = np.random.default_rng(args.seed)
    train, val = synth.Corpus(cfg, "train"), synth.Corpus(cfg, "val")
    print(f"train clips: { {k: len(v) for k, v in train.tts.items()} }, dropped too long: {train.dropped_long}")
    xv, yv = val_set(val, cfg)
    model = WakeNet(width, blocks).to(dev)
    print(f"parameters: {n_params(model)}")
    opt = torch.optim.AdamW(model.parameters(), lr=t["lr"], weight_decay=t["weight_decay"])
    steps = epochs * -(-int(t["positives_per_epoch"] * 3.5) // t["batch"])
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, t["lr"], total_steps=steps)
    out = Path(cfg["paths"]["runs"]) / f"w{width}b{blocks}_s{args.seed}"
    out.mkdir(parents=True, exist_ok=True)
    best, log = -1.0, []
    for ep in range(epochs):
        t0 = time.time()
        audio, y = synth.training_batch(train, rng, t["positives_per_epoch"], t["neg_ratio"], 2.5)
        x, y = features(audio), torch.from_numpy(y)
        perm = torch.randperm(len(y))
        model.train()
        loss_sum = 0.0
        w = torch.tensor([1.0, t["pos_weight"]])
        for i in range(0, len(y), t["batch"]):
            idx = perm[i:i + t["batch"]]
            loss = torch.nn.functional.cross_entropy(model(x[idx].to(dev)), y[idx].to(dev), weight=w.to(dev))
            opt.zero_grad()
            loss.backward()
            opt.step()
            if sched.last_epoch < steps - 1:
                sched.step()
            loss_sum += loss.item() * len(idx)
        score = tpr_at_fpr(predict(model, xv), yv, 0.005)
        log.append({"epoch": ep, "loss": loss_sum / len(y), "val_tpr_at_0.5pct_fpr": score})
        print(f"epoch {ep:2d} loss {loss_sum / len(y):.4f} val TPR@0.5%FPR {score:.3f} ({time.time() - t0:.0f} s)", flush=True)
        if score >= best:
            best = score
            torch.save({"state": {k: v.cpu() for k, v in model.state_dict().items()}, "width": width, "blocks": blocks, "epoch": ep}, out / "model.pt")
    (out / "train_log.json").write_text(json.dumps({"seed": args.seed, "best_val_tpr_at_0.5pct_fpr": best, "log": log}))
    print(f"best val TPR@0.5%FPR {best:.3f} -> {out}/model.pt")


if __name__ == "__main__":
    main()
