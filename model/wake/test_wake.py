"""Unit tests for the wake-word tooling (run: python -I -m pytest model/wake/test_wake.py or python -I model/wake/test_wake.py)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import evaluate  # noqa: E402
import synth  # noqa: E402
import wakelib  # noqa: E402


def test_mfcc_batch_matches_reference():
    rng = np.random.default_rng(0)
    w = rng.normal(0, 2000, (3, wakelib.WINDOW)).astype(np.int16)
    got = wakelib.mfcc_batch(w)
    for i in range(3):
        np.testing.assert_allclose(got[i], wakelib.kws_mfcc.mfcc(w[i]), rtol=1e-3, atol=1e-3)


def test_mfcc_stream_windows_equal_window_mfcc():
    rng = np.random.default_rng(1)
    a = rng.normal(0, 1000, wakelib.WINDOW + 3200).astype(np.int16)
    frames = wakelib.mfcc_stream(a)
    np.testing.assert_allclose(frames[10:10 + 49], wakelib.kws_mfcc.mfcc(a[3200:3200 + wakelib.WINDOW]), rtol=1e-3, atol=1e-3)


def test_debounce_known_answer():
    p = np.array([0, .9, 0, .9, .9, 0, 0, 0, 0, 0, 0, 0, 0, .9, .9, 0])
    assert evaluate.debounce(p, .8, 1, 1, 0) == [1, 3, 4, 13, 14]
    assert evaluate.debounce(p, .8, 3, 2, 10) == [3, 14]   # refractory of 10 windows blocks j=4..13
    assert evaluate.debounce(p, .8, 2, 2, 2) == [4, 14]


def test_wilson_and_poisson():
    lo, hi = evaluate.wilson(5, 10)
    assert 0.18 < lo < 0.25 and 0.75 < hi < 0.82
    lo, hi = evaluate.poisson_ci(0, 2.0)
    assert lo == 0 and abs(hi - 3.689 / 2.0) < 0.01


def test_place_stays_inside_margin():
    rng = np.random.default_rng(2)
    x = np.ones(8000, np.float32)
    for _ in range(50):
        w, s, e = synth.place(x, rng, 320)
        assert s >= 320 and e <= wakelib.WINDOW - 320 and w[s:e].sum() == 8000


def test_reverb_preserves_length_and_energy_order():
    rng = np.random.default_rng(3)
    x = rng.normal(0, 1000, 16000).astype(np.float32)
    y = synth.reverb(x, 0.3, rng)
    assert y.shape == x.shape and 0.3 < synth.rms(y) / synth.rms(x) < 3


if __name__ == "__main__":
    for name, f in list(globals().items()):
        if name.startswith("test_"):
            f()
            print("ok", name)
