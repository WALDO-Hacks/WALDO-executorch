#!/usr/bin/env python3
"""Streaming evaluation of a wake-word model exactly as the firmware runs it: a new 1 s window
(49 MFCC frames) every 200 ms, posterior >= threshold on k of the last n windows, then a
refractory period.

    python -I model/wake/evaluate.py val  --runs w32b4_s1 ...   # grid over thresholds/debounce, writes selection
    python -I model/wake/evaluate.py test --runs w32b4_s1 ...   # reads the VALIDATION selection, test touched once

Trials: TTS utterances of held-out voices placed at a random position in a 3.5 s (positives) or
len+1 s (negatives) augmented stream; real-human negatives: all Speech Commands clips of the
split (shuffled, concatenated) and the split's share of the background-noise recordings.
"""
import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np
import torch
from scipy.stats import beta, chi2

sys.path.insert(0, str(Path(__file__).resolve().parent))
import synth  # noqa: E402
import wakelib  # noqa: E402
from wake_model import WakeNet  # noqa: E402

HOP_FRAMES = 10   # 200 ms


def load_float(run_dir):
    ck = torch.load(Path(run_dir) / "model.pt", weights_only=True)
    m = WakeNet(ck["width"], ck["blocks"])
    m.load_state_dict(ck["state"])
    return m.eval()


def scorer(model):
    """(N, 49, 10) float32 -> (N,) wake posterior."""
    def fn(x, batch=4096):
        with torch.no_grad():
            return np.concatenate([torch.softmax(model(torch.from_numpy(x[i:i + batch]).unsqueeze(-1)), 1)[:, 1].numpy()
                                   for i in range(0, len(x), batch)])
    return fn


def stream_posteriors(fn, audio):
    """Wake posterior at every 200 ms window of a continuous int16 stream: (n_windows,)."""
    feats = wakelib.mfcc_stream(audio)
    starts = np.arange(0, len(feats) - wakelib.FRAMES + 1, HOP_FRAMES)
    if len(starts) == 0:
        return np.zeros(0, np.float32)
    return fn(feats[starts[:, None] + np.arange(wakelib.FRAMES)[None, :]])


def debounce(p, thr, n, k, refractory):
    """Indices of the windows at which a wake event fires (same logic as he/wake_debounce.h)."""
    hit = p >= thr
    events, last = [], -10 ** 9
    for j in range(len(p)):
        if j - last > refractory and hit[max(0, j - n + 1):j + 1].sum() >= k:
            events.append(j)
            last = j
    return events


def wilson(k, n, a=0.05):
    if n == 0:
        return (float("nan"),) * 2
    lo = beta.ppf(a / 2, k, n - k + 1) if k else 0.0
    hi = beta.ppf(1 - a / 2, k + 1, n - k) if k < n else 1.0
    return float(lo), float(hi)


def poisson_ci(k, hours, a=0.05):
    lo = chi2.ppf(a / 2, 2 * k) / 2 if k else 0.0
    return float(lo / hours), float(chi2.ppf(1 - a / 2, 2 * k + 2) / 2 / hours)


def trial_audio(clip, corpus, rng, length, cond=None):
    x, _, _ = synth.place(clip, rng, int(0.3 * wakelib.RATE), n=length)
    return np.round(synth.degrade(x, corpus, rng, cond)).astype(np.int16)


def condition_stream(stream, cond, corpus):
    """The long Speech Commands stream at a fixed point of an evaluation condition: speech scaled
    to the condition's level (mid of its ranges) plus tiled background noise (test noise split)."""
    x = stream.astype(np.float32)
    mid = lambda r: 0.5 * (r[0] + r[1])
    noise = np.resize(corpus.noise, len(x))
    if "speech_rms" in cond:
        x *= mid(cond["speech_rms"]) / synth.active_rms(x[:wakelib.RATE * 600])
        x += noise * (mid(cond["noise_rms"]) / synth.rms(noise))
    else:
        x *= 3000.0 * 10 ** (mid(cond["gain_db"]) / 20) / synth.active_rms(x[:wakelib.RATE * 600])
        x += noise * (synth.rms(x) / synth.rms(noise)) * 10 ** (-mid(cond["snr_db"]) / 20)
    return np.clip(np.round(x), -32768, 32767).astype(np.int16)


def collect(cfg, split, fn, seed, with_conditions=False):
    """Posterior sequences for every trial and stream of a split. Condition results are stored
    under "<kind>@<condition>" (TTS trials) and "sc@<condition>" (the Speech Commands stream)."""
    rng = np.random.default_rng(seed)
    corpus = synth.Corpus(cfg, split)
    reps = cfg["eval"]["positive_trials_per_clip"]
    raw = {"voices": {}, "post": {}, "hours": {}}
    named = {"": None, **{f"@{n}": v for n, v in cfg["conditions"].items()}} if with_conditions else {"": None}
    for suffix, cond in named.items():
        for kind in ("pos", "alone", "conf", "gen", "word"):
            post, voices = [], []
            for clip, voice in zip(corpus.tts[kind], corpus.tts_voice[kind]):
                for _ in range(reps if kind in ("pos", "alone") else 1):
                    n = int(3.5 * wakelib.RATE) if kind in ("pos", "alone") else len(clip) + wakelib.RATE
                    post.append(stream_posteriors(fn, trial_audio(clip, corpus, rng, n, cond)))
                    voices.append(voice)
            raw["post"][kind + suffix], raw["voices"][kind + suffix] = post, voices
    for kind in ("pos", "neg"):   # real recordings (record_real.py): raw, no synthetic augmentation
        raw["post"][f"real_{kind}"] = [stream_posteriors(fn, a) for a in corpus.real[kind]]
        raw["voices"][f"real_{kind}"] = corpus.real_speaker[kind]
    order = rng.permutation(len(corpus.sc))
    stream = np.concatenate([np.asarray(corpus.sc[i]) for i in order])
    raw["hours"]["sc"] = len(stream) / wakelib.RATE / 3600
    raw["post"]["sc"] = [stream_posteriors(fn, stream)]
    for suffix, cond in named.items():
        if cond:
            raw["post"]["sc" + suffix] = [stream_posteriors(fn, condition_stream(stream, cond, corpus))]
            raw["hours"]["sc" + suffix] = raw["hours"]["sc"]
    raw["post"]["noise"] = [stream_posteriors(fn, corpus.noise.astype(np.int16))]
    raw["hours"]["noise"] = len(corpus.noise) / wakelib.RATE / 3600
    raw["dropped_long"] = corpus.dropped_long
    return raw


def rates(raw, thr, deb, cfg):
    """Metrics at one operating point."""
    n, k = deb
    ref = int(cfg["eval"]["debounce"]["refractory_s"] * 1000 / cfg["eval"]["hop_ms"])
    out = {}
    for kind, posts in raw["post"].items():
        if not posts:
            continue
        ev = [len(debounce(p, thr, n, k, ref)) for p in posts]
        if kind.split("@")[0] in ("sc", "noise"):
            c = sum(ev)
            h = raw["hours"][kind]
            out[kind] = {"events": c, "hours": h, "per_hour": c / h, "per_hour_ci95": poisson_ci(c, h)}
        else:
            hit = np.array([e > 0 for e in ev])
            out[kind] = {"trials": len(hit), "triggered": int(hit.sum()), "rate": float(hit.mean()),
                         "wilson95": wilson(int(hit.sum()), len(hit)), "extra_events": int(sum(ev) - hit.sum())}
            voices = np.array(raw["voices"][kind])
            uv = sorted(set(voices))
            per = [hit[voices == v].mean() for v in uv]
            rng = np.random.default_rng(0)
            boot = [np.mean([per[i] for i in rng.integers(len(per), size=len(per))]) for _ in range(2000)]
            out[kind]["voice_bootstrap95"] = [float(np.percentile(boot, 2.5)), float(np.percentile(boot, 97.5))]
            out[kind]["per_voice"] = {v: float(x) for v, x in zip(uv, per)}
    return out


def grid(cfg):
    thr = [round(x, 3) for x in list(np.arange(0.5, 0.95, 0.05)) + [0.95, 0.97, 0.98, 0.99, 0.995, 0.999, 0.9995]]
    d = cfg["eval"]["debounce_grid"]
    return list(itertools.product(thr, [tuple(x) for x in d]))


def select(raws, cfg):
    """On VALIDATION: the operating point with the highest mean TPR over the models subject to
    mean Speech Commands FA/h <= target; ties go to the higher threshold."""
    best = None
    for thr, deb in grid(cfg):
        r = [rates(raw, thr, deb, cfg) for raw in raws]
        fa = np.mean([x["sc"]["per_hour"] for x in r])
        tpr = np.mean([x["pos"]["rate"] for x in r])
        if fa <= cfg["eval"]["fa_target_per_hour"] and (best is None or tpr > best[0] + 1e-9 or (abs(tpr - best[0]) < 1e-9 and thr > best[1])):
            best = (tpr, thr, deb, fa)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("split", choices=["val", "test"])
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--int8", action="store_true", help="evaluate the PT2E int8 model (export_wake.build_quantized)")
    args = ap.parse_args()
    cfg = wakelib.load_config()
    runs = Path(cfg["paths"]["runs"])
    sel_file = runs / "selection.json"
    tag = "int8" if args.int8 else "float"
    if args.split == "test":
        done = runs / f"test_done_{tag}_{'_'.join(args.runs)}.json"
        if done.exists():
            sys.exit(f"{done} exists: the test split is evaluated once per model set; refusing to look again")
        sel = json.loads(sel_file.read_text())
        assert sorted(sel["runs"]) == sorted(args.runs), "test must use the models the VALIDATION selection was made for"
    raws = []
    for name in args.runs:
        model = load_float(runs / name)
        if args.int8:
            import export_wake
            model = export_wake.build_quantized(model, cfg)
        raws.append(collect(cfg, args.split, scorer(model), cfg["seed"] + (0 if args.split == "val" else 1),
                            with_conditions=args.split == "test"))
    if args.split == "val":
        best = select(raws, cfg)
        if best is None:
            sys.exit("no operating point reaches the FA/h target on validation")
        tpr, thr, deb, fa = best
        sel = {"runs": args.runs, "threshold": thr, "debounce_n_k": deb, "val_tpr_mean": tpr, "val_sc_fa_per_hour_mean": fa}
        sel_file.write_text(json.dumps(sel, indent=1))
        print(json.dumps(sel, indent=1))
        for name, raw in zip(args.runs, raws):
            print(name, json.dumps({k: v for k, v in rates(raw, thr, tuple(deb), cfg).items() if k in ("pos", "sc", "conf")}, default=float)[:600])
        return
    res = {"selection": sel, "model": tag, "per_run": {n: rates(r, sel["threshold"], tuple(sel["debounce_n_k"]), cfg)
                                                       for n, r in zip(args.runs, raws)}}
    done.write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
