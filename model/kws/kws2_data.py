"""KWS v2 data: corpora (Speech Commands, ElevenLabs, macOS say), room/mic augmentation,
and vectorised MFCCs that equal mfcc.mfcc() (the firmware's front end) row for row.

All audio is float32 in int16 units at 16 kHz; a window is mfcc.WINDOW = 16000 samples.
"""
import json
import wave
from pathlib import Path

import numpy as np
import scipy.fft
from scipy.signal import butter, fftconvolve, sosfilt

import kws_vocab as V
import mfcc as M

W = M.WINDOW
RATE = M.RATE
LABEL_ID = {l: i for i, l in enumerate(V.LABELS)}
UNK, SIL = LABEL_ID["_unknown_"], LABEL_ID["_silence_"]


# ---------- features ----------

def mfcc_batch(windows):
    """(N, 16000) -> (N, 49, 10) float32, the same numbers as mfcc.mfcc per row."""
    x = np.asarray(windows, np.float32) / 32768.0
    n = x.shape[0]
    idx = np.arange(M.FRAMES)[:, None] * M.STRIDE + np.arange(M.FRAME)[None, :]
    mag = np.abs(scipy.fft.rfft(x[:, idx] * M._HANN, M.FFT, axis=-1, workers=-1)).astype(np.float32)
    e = np.log(np.finfo(np.float32).tiny + mag @ M._FB.T)
    return (e @ M._DCT.T).astype(np.float32).reshape(n, M.FRAMES, M.COEFFS)


def mfcc_stream(audio):
    """Continuous audio -> one MFCC frame per 20 ms (n_frames, 10), the firmware's rolling frames."""
    x = np.asarray(audio, np.float32) / 32768.0
    n = (len(x) - M.FRAME) // M.STRIDE + 1
    if n <= 0:
        return np.zeros((0, M.COEFFS), np.float32)
    idx = np.arange(n)[:, None] * M.STRIDE + np.arange(M.FRAME)[None, :]
    mag = np.abs(scipy.fft.rfft(x[idx] * M._HANN, M.FFT, axis=-1)).astype(np.float32)
    return (np.log(np.finfo(np.float32).tiny + mag @ M._FB.T) @ M._DCT.T).astype(np.float32)


# ---------- io ----------

def read_wav(path):
    with wave.open(str(path)) as w:
        assert w.getframerate() == RATE and w.getsampwidth() == 2 and w.getnchannels() == 1, path
        return np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32)


def trim(x, db=-35):
    a = np.abs(x)
    if a.max() == 0:
        return x[:0]
    env = np.convolve(a, np.ones(160, np.float32) / 160, mode="same")
    i = np.flatnonzero(env > env.max() * 10 ** (db / 20))
    lo, hi = max(0, i[0] - 480), min(len(x), i[-1] + 480)   # keep 30 ms either side
    return x[lo:hi]


def load_corpus(k, skip_texts=()):
    """-> dict split -> list of (audio, label, kind, speaker, source). TTS clips trimmed.
    skip_texts: phrases to leave out (near misses that contain a real label word)."""
    k = Path(k)
    out = {"train": [], "val": [], "test": []}
    el = k / "tts_el"
    if (el / "manifest.jsonl").exists():
        for line in (el / "manifest.jsonl").read_text().splitlines():
            j = json.loads(line)
            p = el / "clips" / f"{j['key']}.pcm"
            if not p.exists() or j["text"] in skip_texts:
                continue
            a = np.frombuffer(p.read_bytes(), np.int16).astype(np.float32)
            a = a if j["kind"] == "sentence" else trim(a)
            if len(a) < 1600:
                continue
            out[j["split"]].append((a, j["label"], j["kind"], "el:" + j["voice_id"], "el"))
    say = k / "tts_say"
    if (say / "manifest.jsonl").exists():
        for line in (say / "manifest.jsonl").read_text().splitlines():
            j = json.loads(line)
            if j["text"] in skip_texts:
                continue
            a = read_wav(j["path"])
            a = a if j["kind"] == "sentence" else trim(a)
            if len(a) < 1600:
                continue
            out[j["split"]].append((a, j["label"], j["kind"], "say:" + j["voice"].split(" (")[0], "say"))
    return out


def sc_index(k):
    """Speech Commands v0.02: dict split -> list of (path, word, speaker). Official speaker-disjoint lists."""
    sc = Path(k) / "sc"
    lists = {s: set((sc / f"{n}_list.txt").read_text().split()) for s, n in (("val", "validation"), ("test", "testing"))}
    out = {"train": [], "val": [], "test": []}
    for p in sorted(sc.glob("*/*.wav")):
        if p.parent.name.startswith("_"):
            continue
        rel = f"{p.parent.name}/{p.name}"
        s = "val" if rel in lists["val"] else "test" if rel in lists["test"] else "train"
        out[s].append((str(p), p.parent.name, p.name.split("_")[0]))
    return out


def noise_bank(k):
    """Background noise files (Speech Commands), split by time: first 70 % train, next 15 % val, last 15 % test."""
    files = sorted((Path(k) / "sc" / "_background_noise_").glob("*.wav"))
    bank = {"train": [], "val": [], "test": []}
    for f in files:
        a = read_wav(f)
        n = len(a)
        bank["train"].append(a[:int(.7 * n)])
        bank["val"].append(a[int(.7 * n):int(.85 * n)])
        bank["test"].append(a[int(.85 * n):])
    return bank


# ---------- augmentation ----------

def rms(x):
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64)) + 1e-9))


def active_rms(x):
    a = np.abs(x)
    m = a > 0.05 * a.max() if a.max() > 0 else a >= 0
    return rms(x[m])


def resample(x, f):
    n = max(2, int(len(x) / f))
    return np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x).astype(np.float32)


def place(x, rng, n=W, margin=320):
    """Speech fully inside an n-sample window at a random offset (cropped if longer)."""
    if len(x) >= n - 2 * margin:
        s = int(rng.integers(0, max(1, len(x) - n + 1)))
        out = x[s:s + n]
        return np.pad(out, (0, n - len(out)))
    s = int(rng.integers(margin, n - len(x) - margin + 1))
    out = np.zeros(n, np.float32)
    out[s:s + len(x)] = x
    return out


def room(x, rng):
    """Room at 30-150 cm: exponential-decay reverb tail, direct-to-reverberant ratio by distance."""
    rt60 = rng.uniform(0.15, 0.6)
    d = rng.uniform(0.3, 1.5)
    drr_db = 12.0 - 12.0 * (d - 0.3) / 1.2
    n = int(rt60 * RATE)
    t = np.arange(n) / RATE
    tail = rng.standard_normal(n).astype(np.float32) * np.exp(-6.9 * t / rt60).astype(np.float32)
    tail[:int(0.003 * RATE)] = 0
    tail *= 10 ** (-drr_db / 20) / max(np.sqrt(np.sum(tail ** 2)), 1e-9)
    ir = tail
    ir[0] = 1.0
    return fftconvolve(x, ir)[:len(x)].astype(np.float32)


def mic(x, rng):
    """PDM mic and front-end colouring: high-pass, sometimes low-pass, a tilt."""
    sos = butter(2, rng.uniform(70, 250), "highpass", fs=RATE, output="sos")
    x = sosfilt(sos, x)
    if rng.random() < 0.5:
        x = sosfilt(butter(2, rng.uniform(3500, 7500), "lowpass", fs=RATE, output="sos"), x)
    if rng.random() < 0.3:   # a resonance
        f0 = rng.uniform(800, 4000)
        x = x + rng.uniform(0.1, 0.4) * sosfilt(butter(1, [f0 * 0.8, f0 * 1.2], "bandpass", fs=RATE, output="sos"), x)
    return x.astype(np.float32)


def noise_window(bank, rng, n=W, babble_src=None):
    r = rng.random()
    if babble_src is not None and r < 0.2:
        out = np.zeros(n, np.float32)
        for _ in range(rng.integers(1, 4)):
            c = babble_src[rng.integers(len(babble_src))]
            out[:len(c)] += c[:n] * rng.uniform(0.3, 1.0)
        return np.roll(out, int(rng.integers(n)))
    if r < 0.3:
        w = rng.standard_normal(n).astype(np.float32)
        if rng.random() < 0.5:   # pinkish
            w = np.cumsum(w).astype(np.float32)
            w -= np.convolve(w, np.ones(400) / 400, mode="same").astype(np.float32)
        return w
    f = bank[rng.integers(len(bank))]
    if len(f) <= n:   # long streams (evaluation): loop the file
        f = np.tile(f, n // len(f) + 2)
    s = int(rng.integers(0, len(f) - n))
    return f[s:s + n].copy()


# Training augmentation odds (kws2_featurize.py --aug can override).
AUG = {"room": 0.4, "mic": 0.6, "noise": 0.7, "board": 0.2, "dc": 0.3}


def degrade(x, rng, bank, babble_src=None, cond=None):
    """Training augmentation (cond None) or a fixed evaluation condition:
    cond = {"snr": dB or None, "gain_db": dB, "room": bool, "board": bool}."""
    n = len(x)
    if cond is None:
        if rng.random() < AUG["room"]:
            x = room(x, rng)
        if rng.random() < AUG["mic"]:
            x = mic(x, rng)
        nz = noise_window(bank, rng, n, babble_src) if rng.random() < AUG["noise"] else None
        if rng.random() < AUG["board"]:   # the board's measured regime: speech 500-1200 rms over 165-800
            if active_rms(x) > 0:
                x = x * (rng.uniform(500, 1200) / active_rms(x))
            if nz is not None and rms(nz) > 0:
                x = x + nz * (rng.uniform(165, 800) / rms(nz))
        else:
            pk = np.abs(x).max()
            if pk > 0:
                x = x * (16000 / pk) * 10 ** (rng.uniform(-30, 3) / 20)
            if nz is not None and rms(nz) > 0 and active_rms(x) > 1:
                x = x + nz * (active_rms(x) / rms(nz)) * 10 ** (-rng.uniform(3, 30) / 20)
        if rng.random() < AUG["dc"]:
            x = x + rng.uniform(-300, 300)
    else:
        if cond.get("room"):
            x = mic(room(x, rng), rng)
        pk = np.abs(x).max()
        if cond.get("board"):
            if active_rms(x) > 0:
                x = x * (rng.uniform(500, 1200) / active_rms(x))
            nz = noise_window(bank, rng, n)
            x = x + nz * (rng.uniform(165, 800) / max(rms(nz), 1e-6))
        else:
            if pk > 0:
                x = x * (16000 / pk) * 10 ** (cond.get("gain_db", 0) / 20)
            if cond.get("snr") is not None:
                nz = noise_window(bank, rng, n)
                x = x + nz * (active_rms(x) / max(rms(nz), 1e-6)) * 10 ** (-cond["snr"] / 20)
    # the PDM mics never give digital silence: a little noise always (and the int8 input range stays sane)
    x = x + rng.normal(0, rng.uniform(1, 6) if cond is None else 3.0, len(x)).astype(np.float32)
    return np.clip(x, -32768, 32767).astype(np.float32)


def silence_window(bank, rng):
    nz = noise_window(bank, rng)
    return np.clip(nz * (10 ** (rng.uniform(-70, -20) / 20) * 32767 / max(rms(nz), 1e-6)), -32768, 32767)


def dither(x, std=3.0):
    """Clean audio as a microphone delivers it: never exactly zero (int16 rms ~3 counts)."""
    return (x + np.random.default_rng(len(x)).normal(0, std, len(x))).astype(np.float32)
