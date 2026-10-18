"""MFCC features for MicroNet KWS, the same maths as Alif's ML Embedded Evaluation Kit
(source/lib/mlek/common/Mfcc.cc with MicroNetKwsMfcc.hpp) and src/waldo/kws_mfcc.c.

1 s of 16 kHz audio -> 49 frames of 640 samples (40 ms) every 320 (20 ms) -> per frame:
Hann window, 1024-point FFT, magnitude, 40 HTK mel bands (20-4000 Hz), natural log, DCT-II
(orthonormal scale, no special first term) -> first 10 coefficients.
"""
import numpy as np

RATE, FRAME, STRIDE, FFT, BANDS, COEFFS, FRAMES = 16000, 640, 320, 1024, 40, 10, 49
WINDOW = FRAMES * STRIDE + (FRAME - STRIDE)   # 16000 samples


def _mel(f):
    return 1127.0 * np.log(1.0 + f / 700.0)


def mel_filterbank():
    bins = FFT // 2
    lo, hi = _mel(20.0), _mel(4000.0)
    delta = (hi - lo) / (BANDS + 1)
    mel = _mel(np.arange(bins) * RATE / FFT)
    fb = np.zeros((BANDS, bins + 1), np.float32)
    for b in range(BANDS):
        l, c, r = lo + b * delta, lo + (b + 1) * delta, lo + (b + 2) * delta
        up = (mel > l) & (mel <= c)
        down = (mel > c) & (mel < r)
        fb[b, :bins][up] = ((mel - l) / (c - l))[up]
        fb[b, :bins][down] = ((r - mel) / (r - c))[down]
    return fb


def dct_matrix():
    n = np.arange(BANDS)
    return np.array([np.sqrt(2.0 / BANDS) * np.cos((n + 0.5) * k * np.pi / BANDS)
                     for k in range(COEFFS)], np.float32)


_FB, _DCT = mel_filterbank(), dct_matrix()
_HANN = (0.5 - 0.5 * np.cos(np.arange(FRAME) * 2 * np.pi / FRAME)).astype(np.float32)


def mfcc(audio):
    """audio: int16 array of WINDOW samples -> (49, 10) float32."""
    x = audio.astype(np.float32) / 32768.0
    out = np.empty((FRAMES, COEFFS), np.float32)
    for i in range(FRAMES):
        f = x[i * STRIDE:i * STRIDE + FRAME] * _HANN
        mag = np.abs(np.fft.rfft(f, FFT)).astype(np.float32)          # 513 bins
        e = np.log(np.finfo(np.float32).tiny + _FB @ mag)
        out[i] = _DCT @ e
    return out


def read_wav(path):
    import wave
    with wave.open(str(path)) as w:
        assert w.getframerate() == RATE and w.getsampwidth() == 2 and w.getnchannels() == 1
        return np.frombuffer(w.readframes(w.getnframes()), np.int16)


def windows(audio, step=WINDOW // 2):
    """1 s windows every step samples (zero-padded at the end), like the MLEK sample."""
    n = max(1, int(np.ceil((len(audio) - WINDOW) / step)) + 1)
    pad = np.zeros((n - 1) * step + WINDOW, np.int16)
    pad[:len(audio)] = audio
    return [pad[i * step:i * step + WINDOW] for i in range(n)]
