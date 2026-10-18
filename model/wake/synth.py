"""Waveform-domain data synthesis for the wake word: corpus loading, augmentation and the
1 s example builders used by training (random, augmented) and evaluation (fixed seeds).

All audio is int16-range float32 at 16 kHz; windows are wakelib.WINDOW = 16000 samples.
"""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.signal import fftconvolve

import wakelib

W = wakelib.WINDOW


class Corpus:
    """TTS clips (trimmed), Speech Commands memmaps and background noise for one split."""

    def __init__(self, cfg, split):
        self.cfg, self.split = cfg, split
        root, cache = Path(cfg["paths"]["tts"]), Path(cfg["paths"]["cache"])
        a = cfg["audio"]
        self.tts = {k: [] for k in ("pos", "alone", "conf", "gen", "word")}
        self.tts_voice = {k: [] for k in self.tts}
        self.dropped_long = 0
        with open(root / "index.csv") as f:
            for r in csv.DictReader(f):
                if r["split"] != split:
                    continue
                x = wakelib.trim(wakelib.read_wav(root / r["file"]), a["trim_db"]).astype(np.float32)
                if len(x) < a["min_phrase_s"] * wakelib.RATE:
                    continue
                if r["kind"] in ("pos", "alone") and len(x) > a["max_phrase_s"] * wakelib.RATE:
                    self.dropped_long += 1
                    continue
                self.tts[r["kind"]].append(x)
                self.tts_voice[r["kind"]].append(r["voice"])
        self.real = {"pos": [], "neg": []}   # record_real.py recordings of this split's speakers
        self.real_speaker = {"pos": [], "neg": []}
        real_dir = Path(cfg["paths"]["real"])
        if real_dir.is_dir():
            assigned = {s: sp for sp in ("train", "val") for s in cfg["real"][f"{sp}_speakers"]}
            for d in sorted(p for p in real_dir.iterdir() if p.is_dir()):
                if assigned.get(d.name, "test") != split:
                    continue
                for f in sorted(d.glob("*.wav")):
                    kind = "pos" if f.name.startswith("pos_") else "neg" if f.name.startswith("neg_") else None
                    if kind:
                        self.real[kind].append(wakelib.read_wav(f))
                        self.real_speaker[kind].append(d.name)
        self.sc = np.load(cache / f"sc_{split}.npy", mmap_mode="r")
        meta = json.loads((cache / "sc_meta.json").read_text())[split]
        self.sc_words, self.sc_speakers = np.array(meta["words"]), np.array(meta["speakers"])
        self.noise = np.load(cache / f"noise_{split}.npy").astype(np.float32)
        self.noise_lens = np.load(cache / f"noise_{split}_lens.npy")


def resample(x, factor):
    """Speed change by `factor` (>1 = shorter and higher) with linear interpolation."""
    n = int(len(x) / factor)
    return np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x).astype(np.float32)


def reverb(x, rt60, rng):
    """Convolve with an exponentially decaying noise impulse response (direct path kept)."""
    n = int(min(rt60, 0.6) * wakelib.RATE)
    t = np.arange(n) / wakelib.RATE
    ir = rng.standard_normal(n).astype(np.float32) * np.exp(-6.9 * t / rt60)
    ir[0] = 1.0
    ir /= np.sqrt(np.sum(ir ** 2))
    return fftconvolve(x, ir)[:len(x)].astype(np.float32)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64)) + 1e-9))


def noise_crop(corpus, n, rng):
    """n samples of background noise from one file (no seam across files)."""
    ends = np.cumsum(corpus.noise_lens)
    i = rng.integers(len(ends))
    lo, hi = (ends[i - 1] if i else 0), ends[i]
    start = rng.integers(lo, max(lo + 1, hi - n))
    out = corpus.noise[start:start + n]
    return np.pad(out, (0, n - len(out)))


def babble(corpus, n, rng):
    """1-3 random Speech Commands clips mixed: speech-like interference."""
    out = np.zeros(n, np.float32)
    for _ in range(rng.integers(1, 4)):
        clip = corpus.sc[rng.integers(len(corpus.sc))][:n].astype(np.float32) * 10 ** rng.uniform(-0.5, 0)
        s = int(rng.integers(0, max(1, n - len(clip) + 1)))
        out[s:s + len(clip)] += clip
    return out


def place(x, rng, margin, n=W):
    """x inside an n-sample window of zeros at a random offset at least `margin` samples from
    the edges; longer clips are randomly cropped. Returns (window, start, stop)."""
    if len(x) > n:
        s = rng.integers(len(x) - n + 1)
        return x[s:s + n].copy(), 0, n
    lo, hi = min(margin, n - len(x)), max(min(margin, n - len(x)), n - len(x) - margin)
    s = int(rng.integers(lo, hi + 1))
    out = np.zeros(n, np.float32)
    out[s:s + len(x)] = x
    return out, s, s + len(x)


def active_rms(x):
    """RMS over the samples above 5 % of the peak (the speech, not the padding)."""
    a = np.abs(x)
    m = a > 0.05 * a.max() if a.max() > 0 else a > 0
    return rms(x[m]) if m.any() else 0.0


def degrade(x, corpus, rng, cond=None):
    """Reverb, background noise or babble, gain and DC offset; clipped to int16 range.

    cond (eval conditions, config eval.conditions) overrides the training distribution with
    explicit ranges: gain_db relative to a speech peak of 0.5 full scale, snr_db, or the
    absolute speech_rms / noise_rms in int16 counts (the board's measured regime).
    """
    aug = dict(corpus.cfg["augment"])
    if cond:
        aug.update(cond)
        aug["noise_prob"], aug["board_prob"] = 1.0, float(("speech_rms" in cond))
    n = len(x)
    if rng.random() < aug["reverb_prob"]:
        x = reverb(x, rng.uniform(*aug["reverb_rt60_s"]), rng)
    peak = np.abs(x).max()
    bed = None
    if rng.random() < aug["noise_prob"]:
        bed = babble(corpus, n, rng) if rng.random() < 0.25 else noise_crop(corpus, n, rng)
    if peak > 0 and rng.random() < aug["board_prob"]:
        # the board's regime: speech ~500-1200 counts rms over a 165-800 counts noise floor
        x = x * (rng.uniform(*aug["speech_rms"]) / max(active_rms(x), 1e-6))
        if bed is not None and rms(bed) > 0:
            x = x + bed * (rng.uniform(*aug["noise_rms"]) / rms(bed))
    else:
        if peak > 0:
            x = x * (0.5 * 32767 / peak)
        x = x * 10 ** (rng.uniform(*aug["gain_db"]) / 20)
        if bed is not None and rms(bed) > 0 and rms(x) > 1:
            x = x + bed * (rms(x) / rms(bed)) * 10 ** (-rng.uniform(*aug["snr_db"]) / 20)
    x = x + rng.uniform(-aug["dc_max"], aug["dc_max"]) * (rng.random() < 0.3)
    return np.clip(x, -32768, 32767).astype(np.float32)


def quiet_noise(corpus, rng):
    """A noise-only window at a random level."""
    n = noise_crop(corpus, W, rng)
    level = 10 ** (rng.uniform(-60, -15) / 20) * 32767
    return n * (level / max(rms(n), 1e-6))


def sc_clip(corpus, i):
    return np.asarray(corpus.sc[i], np.float32)


def sc_mix(corpus, rng, trim_db):
    parts = []
    for _ in range(rng.integers(2, 4)):
        x = wakelib.trim(corpus.sc[rng.integers(len(corpus.sc))], trim_db).astype(np.float32)
        if len(x):
            parts += [x, np.zeros(int(rng.uniform(0.0, 0.25) * wakelib.RATE), np.float32)]
    x = np.concatenate(parts) if parts else np.zeros(W, np.float32)
    return place(x, rng, 0)[0]


def positive(corpus, rng, speed=True, cond=None):
    aug = corpus.cfg["augment"]
    margin = int(aug["edge_margin_s"] * wakelib.RATE)
    for _ in range(20):
        if corpus.real["pos"] and rng.random() < 0.3:   # real recordings, when this split has any
            x = wakelib.trim(corpus.real["pos"][rng.integers(len(corpus.real["pos"]))], corpus.cfg["audio"]["trim_db"]).astype(np.float32)
        else:
            x = corpus.tts["pos"][rng.integers(len(corpus.tts["pos"]))]
        if speed:
            x = resample(x, rng.uniform(*aug["speed"]))
        if len(x) <= W - 2 * margin:
            return degrade(place(x, rng, margin)[0], corpus, rng, cond)
    raise RuntimeError("no positive fits the window")


def negative(corpus, rng, kind):
    """One negative 1 s window of the given kind (config train.neg_ratio keys)."""
    margin = int(corpus.cfg["augment"]["edge_margin_s"] * wakelib.RATE)
    trim_db = corpus.cfg["audio"]["trim_db"]
    if kind == "sc_words":
        return degrade(np.roll(sc_clip(corpus, rng.integers(len(corpus.sc))), int(rng.integers(-4000, 4000))),
                       corpus, rng)
    if kind == "sc_mix":
        return degrade(sc_mix(corpus, rng, trim_db), corpus, rng)
    if kind in ("tts_conf", "tts_gen"):
        pool = corpus.tts["conf"] if kind == "tts_conf" else corpus.tts["gen"] + corpus.tts["word"] + [
            wakelib.trim(x, corpus.cfg["audio"]["trim_db"]).astype(np.float32) for x in corpus.real["neg"]]
        return degrade(place(pool[rng.integers(len(pool))], rng, margin)[0], corpus, rng)
    if kind == "noise":
        return quiet_noise(corpus, rng) if rng.random() < 0.7 else degrade(babble(corpus, W, rng), corpus, rng)
    if kind == "partial":   # the head of a Hey Waldo ("hey", "hey wa"): not the wake word
        x = corpus.tts["pos"][rng.integers(len(corpus.tts["pos"]))]
        return degrade(place(x[:int(len(x) * rng.uniform(0.25, 0.6))], rng, margin)[0], corpus, rng)
    raise KeyError(kind)


def training_batch(corpus, rng, n_pos, ratios, neg_per_pos):
    """(audio (N, 16000) float32, labels (N,) int64: 1 = wake)."""
    kinds = list(ratios)
    p = np.array([ratios[k] for k in kinds], float)
    p /= p.sum()
    n_neg = int(n_pos * neg_per_pos)
    audio = [positive(corpus, rng) for _ in range(n_pos)]
    audio += [negative(corpus, rng, kinds[k]) for k in rng.choice(len(kinds), n_neg, p=p)]
    y = np.r_[np.ones(n_pos, np.int64), np.zeros(n_neg, np.int64)]
    return np.stack(audio), y
