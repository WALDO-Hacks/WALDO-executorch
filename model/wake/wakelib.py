"""Shared helpers for the wake-word scripts: config, wav IO, voice splits, MFCC batches.

The front end is model/kws/mfcc.py (the firmware's src/waldo/kws_mfcc.c does the same maths);
`mfcc_batch` only vectorises it over many windows and is tested against `mfcc.mfcc`.
"""
import os
import re
import sys
import wave
from pathlib import Path

import numpy as np
import scipy.fft
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT / "model" / "kws"))
import mfcc as kws_mfcc  # noqa: E402

RATE = kws_mfcc.RATE
WINDOW = kws_mfcc.WINDOW          # samples, 1 s
HOP = kws_mfcc.STRIDE             # samples, 20 ms between MFCC frames
FRAMES, COEFFS = kws_mfcc.FRAMES, kws_mfcc.COEFFS


def load_config(path=HERE / "config.yaml"):
    cfg = yaml.safe_load(Path(path).read_text())
    for k, v in cfg["paths"].items():
        p = Path(os.path.expandvars(v)).expanduser()
        cfg["paths"][k] = str(p if p.is_absolute() else (ROOT / p).resolve())
    return cfg


def read_wav(path):
    """16 kHz mono int16 wav -> int16 (n,). Anything else is an error, not a silent resample."""
    with wave.open(str(path)) as w:
        if (w.getframerate(), w.getsampwidth(), w.getnchannels()) != (RATE, 2, 1):
            raise ValueError(f"{path}: want 16 kHz mono int16, got {w.getframerate()} Hz "
                             f"{w.getnchannels()} ch {8 * w.getsampwidth()} bit")
        return np.frombuffer(w.readframes(w.getnframes()), np.int16)


def write_wav(path, audio):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(np.asarray(audio, np.int16).tobytes())


def slug(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def voice_groups(cfg):
    """Voice names without the (UK)/(US) tag: the two variants of one name share a split."""
    return sorted({re.sub(r"\s*\(.*\)", "", v) for v in cfg["tts"]["voices"]})


def split_of_group(cfg):
    """{group: 'train'|'val'|'test'}: a seeded shuffle of the voice names, never tuned on results."""
    groups = voice_groups(cfg)
    order = np.random.default_rng(cfg["seed"]).permutation(len(groups))
    nv, nt = cfg["tts"]["split_groups"]["val"], cfg["tts"]["split_groups"]["test"]
    out = {}
    for rank, gi in enumerate(order):
        out[groups[gi]] = "test" if rank < nt else "val" if rank < nt + nv else "train"
    return out


def split_of_voice(cfg, voice):
    return split_of_group(cfg)[re.sub(r"\s*\(.*\)", "", voice)]


def trim(audio, trim_db):
    """Cut leading and trailing samples more than trim_db below the clip's peak (smoothed 10 ms)."""
    a = np.abs(audio.astype(np.float32))
    if a.max() == 0:
        return audio[:0]
    env = np.convolve(a, np.ones(160, np.float32) / 160, mode="same")
    idx = np.flatnonzero(env > env.max() * 10 ** (trim_db / 20))
    return audio[idx[0]:idx[-1] + 1]


def mfcc_batch(windows):
    """(N, 16000) int16 -> (N, 49, 10) float32, the same numbers as kws mfcc.mfcc per row."""
    w = np.asarray(windows)
    n = w.shape[0]
    x = w.astype(np.float32) / 32768.0
    idx = np.arange(FRAMES)[:, None] * kws_mfcc.STRIDE + np.arange(kws_mfcc.FRAME)[None, :]
    frames = x[:, idx] * kws_mfcc._HANN                              # (N, 49, 640)
    mag = np.abs(scipy.fft.rfft(frames, kws_mfcc.FFT, axis=-1, workers=-1)).astype(np.float32)   # (N, 49, 513)
    e = np.log(np.finfo(np.float32).tiny + mag @ kws_mfcc._FB.T)     # (N, 49, 40)
    return (e @ kws_mfcc._DCT.T).astype(np.float32).reshape(n, FRAMES, COEFFS)


def mfcc_stream(audio):
    """Continuous int16 audio (n,) -> one MFCC frame per 20 ms: (n_frames, 10) float32.

    Frame i covers samples [i*320, i*320+640), exactly the firmware's rolling frames.
    """
    n = (len(audio) - kws_mfcc.FRAME) // kws_mfcc.STRIDE + 1
    if n <= 0:
        return np.zeros((0, COEFFS), np.float32)
    out = np.empty((n, COEFFS), np.float32)
    x = audio.astype(np.float32) / 32768.0
    for s in range(0, n, 2048):
        e = min(n, s + 2048)
        idx = np.arange(s, e)[:, None] * kws_mfcc.STRIDE + np.arange(kws_mfcc.FRAME)[None, :]
        mag = np.abs(scipy.fft.rfft(x[idx] * kws_mfcc._HANN, kws_mfcc.FFT, axis=-1, workers=-1)).astype(np.float32)
        out[s:e] = (np.log(np.finfo(np.float32).tiny + mag @ kws_mfcc._FB.T) @ kws_mfcc._DCT.T)
    return out
