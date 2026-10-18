#!/usr/bin/env python3
"""Score WALDO's keyword spotter on many synthetic speakers (ElevenLabs text to speech).

    ELEVENLABS_API_KEY=... <python with ai_edge_litert, numpy> tools/kws_tts_test.py [--out kws_tts]

Every available voice says each command word, plus some ordinary sentences (none of the
command words) for false triggers. The clips are cached in --out, so a second run costs
nothing. Each clip is scored with the board's listening logic (tools/kws_live.py) clean, at
two noise levels and quieter, for a range of thresholds. Synthetic voices are not the
DevKit's microphones: use this to rank words and pick the threshold, not as the final number.
"""
import argparse
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kws_live as K  # noqa: E402

API = "https://api.elevenlabs.io/v1"
WORDS = ["off", "on", "go", "stop", "left", "right", "up", "down", "no", "yes"]
SENTENCES = [
    "Hi everyone, this is our hackathon project.",
    "It runs entirely on the microcontroller, with no cloud.",
    "The camera sends compressed frames over a serial link.",
    "Let me show you what happens when somebody walks in.",
    "We measured the power draw with a battery pack.",
    "Thanks for coming by, do you have any questions?",
]
CONDITIONS = {"clean": (1.0, None), "noise 20 dB": (1.0, 20), "noise 10 dB": (1.0, 10), "quiet x0.1": (0.1, None)}
THRESHOLDS = (0.5, 0.6, 0.7, 0.8, 0.9)


def api(path, key, body=None):
    req = urllib.request.Request(API + path, data=json.dumps(body).encode() if body else None,
                                 headers={"xi-api-key": key, "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read()


def tts(key, voice, text, path):
    if path.exists():
        return np.frombuffer(path.read_bytes(), np.int16)
    for attempt in range(4):
        try:
            pcm = api(f"/text-to-speech/{voice}?output_format=pcm_16000", key,
                      {"text": text, "model_id": "eleven_multilingual_v2"})
            path.write_bytes(pcm)
            return np.frombuffer(pcm, np.int16)
        except Exception as e:  # rate limits: back off and retry
            print(f"  retry {voice} '{text[:20]}': {e}")
            time.sleep(2 + 3 * attempt)
    return None


def condition(a, gain, snr, rng):
    x = a.astype(np.float32) * gain
    if snr is not None:
        p = np.mean(x[np.abs(x) > 0.05 * np.abs(x).max()] ** 2)
        x = x + rng.normal(0, np.sqrt(p / 10 ** (snr / 10)), x.shape)
    return np.clip(x, -32768, 32767).astype(np.int16)


def pad(a):
    z = np.zeros(K.M.WINDOW // 2, np.int16)
    return np.concatenate([z, a, z])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("kws_tts"))
    ap.add_argument("--voices", type=int, default=40)
    args = ap.parse_args()
    key = os.environ["ELEVENLABS_API_KEY"]
    args.out.mkdir(parents=True, exist_ok=True)
    voices = json.loads(api("/voices", key))["voices"][:args.voices]
    print(f"{len(voices)} voices")
    model, rng = K.Model(), np.random.default_rng(0)

    clips = []   # (voice name, word, audio)
    for v in voices:
        for w in WORDS:
            a = tts(key, v["voice_id"], w.capitalize() + ".", args.out / f"{v['voice_id']}_{w}.pcm")
            if a is not None:
                clips.append((v["name"].split(" - ")[0], w, a))
        print(f"  {v['name'].split(' - ')[0]}: done")
    talk = [tts(key, v["voice_id"], s, args.out / f"{v['voice_id']}_s{i}.pcm")
            for i, s in enumerate(SENTENCES) for v in voices[:: max(1, len(voices) // 8)]]
    talk = [t for t in talk if t is not None]
    talk_s = sum(len(t) for t in talk) / K.M.RATE

    results = {}
    for cname, (gain, snr) in CONDITIONS.items():
        print(f"\n{cname}: {len(clips)} command clips, {talk_s:.0f} s of other speech")
        print(f"{'threshold':>9} {'hit %':>6} {'wrong':>6} {'missed':>6} {'false/min':>9}")
        for th in THRESHOLDS:
            hit = wrong = miss = 0
            per_word = {}
            for name, w, a in clips:
                got = [x for _, x, _ in K.listen_offline(model, pad(condition(a, gain, snr, rng)), th)]
                ok = w in got
                hit += ok
                wrong += (not ok) and bool(got)
                miss += not got
                per_word.setdefault(w, []).append(ok)
            false = sum(len(K.listen_offline(model, pad(condition(t, gain, snr, rng)), th)) for t in talk)
            print(f"{th:>9.2f} {100 * hit / len(clips):>6.1f} {wrong:>6} {miss:>6} {60 * false / talk_s:>9.2f}")
            results[(cname, th)] = per_word
    print(f"\nper word at {K.THRESHOLD} (src/kws.cpp), % heard right:")
    print(f"{'word':6s} " + " ".join(f"{c:>12s}" for c in CONDITIONS))
    for w in WORDS:
        print(f"{w:6s} " + " ".join(f"{100 * np.mean(results[(c, K.THRESHOLD)][w]):>12.0f}" for c in CONDITIONS))


if __name__ == "__main__":
    main()
