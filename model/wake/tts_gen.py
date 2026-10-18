#!/usr/bin/env python3
"""Generate the TTS corpus with macOS `say` into <paths.tts>/<voice>/*.wav (16 kHz mono int16)
and <paths.tts>/index.csv (voice, split, kind, phrase, rate, pitch, file).

kinds: pos (Hey Waldo variants), alone ("waldo"), conf (confusable negatives), gen (ordinary
sentences), word (the 10 command words). Voices are split by name (wakelib.split_of_group).
Licence of macOS TTS output for training a model: UNVERIFIED (see documentation/wake-word.md).

    python -I model/wake/tts_gen.py
"""
import csv
import hashlib
import itertools
import subprocess
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wakelib  # noqa: E402


def say(job):
    voice, kind, phrase, rate, pbas, out = job
    text = phrase if pbas is None else f"[[pbas {pbas}]] {phrase}"
    if not out.exists():
        subprocess.run(["say", "-v", voice, "-r", str(rate), "-o", str(out),
                        "--data-format=LEI16@16000", text], check=True)
    return job


def jobs(cfg, root):
    t = cfg["tts"]
    rng = __import__("numpy").random.default_rng(cfg["seed"])
    for voice in t["voices"]:
        d = root / wakelib.slug(voice)
        d.mkdir(parents=True, exist_ok=True)

        def job(kind, phrase, rate, pbas):
            h = hashlib.sha1(f"{voice}|{kind}|{phrase}|{rate}|{pbas}".encode()).hexdigest()[:10]
            return (voice, kind, phrase, rate, pbas, d / f"{kind}_{h}.wav")

        for ph, r, p in itertools.product(t["positives"], t["rates_wpm_positive"], t["pitch_base"]):
            yield job("pos", ph, r, p)
        for ph, r, p in itertools.product(t["waldo_alone"], t["rates_wpm_positive"], t["pitch_base"]):
            yield job("alone", ph, r, p)
        for kind, phrases in (("conf", t["confusable"]), ("gen", t["sentences"]), ("word", t["words"])):
            for ph, r in itertools.product(phrases, t["rates_wpm_negative"]):
                yield job(kind, ph, r, t["pitch_base"][rng.integers(len(t["pitch_base"]))])


def main():
    cfg = wakelib.load_config()
    root = Path(cfg["paths"]["tts"])
    root.mkdir(parents=True, exist_ok=True)
    todo = list(jobs(cfg, root))
    with Pool(8) as pool:
        done = list(pool.imap(say, todo, chunksize=8))
    # drop duplicate audio (a pitch setting some voices ignore), keep the first
    seen, rows = set(), []
    for voice, kind, phrase, rate, pbas, out in done:
        digest = hashlib.sha1(out.read_bytes()[44:]).hexdigest()
        key = (voice, kind, phrase, digest)
        if key in seen:
            continue
        seen.add(key)
        rows.append([voice, wakelib.split_of_voice(cfg, voice), kind, phrase, rate, pbas or "", out.relative_to(root)])
    with open(root / "index.csv", "w", newline="") as f:
        csv.writer(f).writerows([["voice", "split", "kind", "phrase", "rate_wpm", "pbas", "file"]] + rows)
    print(f"{len(done)} utterances, {len(rows)} after removing identical audio -> {root}/index.csv")
    for s in ("train", "val", "test"):
        print(s, {k: sum(1 for r in rows if r[1] == s and r[2] == k) for k in ("pos", "alone", "conf", "gen", "word")})


if __name__ == "__main__":
    main()
