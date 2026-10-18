#!/usr/bin/env python3
"""macOS `say` clips for KWS v2 (16 kHz int16 wav), cached: DIR/<voice>/<sha1>.wav + DIR/manifest.jsonl.

    python -I model/kws/tts_say.py --out DIR

Voices split by name (UK/US variants of one name share a split).
"""
import argparse
import hashlib
import json
import random
import re
import subprocess
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kws_vocab as V  # noqa: E402

VOICES = ["Albert", "Daniel", "Eddy (English (UK))", "Eddy (English (US))", "Flo (English (UK))",
          "Flo (English (US))", "Fred", "Grandma (English (UK))", "Grandma (English (US))",
          "Grandpa (English (UK))", "Grandpa (English (US))", "Junior", "Karen", "Kathy", "Moira",
          "Ralph", "Reed (English (UK))", "Reed (English (US))", "Rishi", "Rocko (English (UK))",
          "Rocko (English (US))", "Samantha", "Sandy (English (UK))", "Sandy (English (US))",
          "Shelley (English (UK))", "Shelley (English (US))", "Tessa"]
TEST = {"Moira", "Rishi", "Tessa", "Grandpa", "Shelley"}
VAL = {"Daniel", "Flo", "Kathy"}


def group(v):
    return re.sub(r"\s*\(.*\)", "", v)


def split(v):
    g = group(v)
    return "test" if g in TEST else "val" if g in VAL else "train"


def run(job):
    if not Path(job["path"]).exists():
        text = job["text"] if job["pbas"] is None else f"[[pbas {job['pbas']}]] {job['text']}"
        subprocess.run(["say", "-v", job["voice"], "-r", str(job["rate"]), "-o", job["path"],
                        "--data-format=LEI16@16000", text], check=True)
    return job


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    rng = random.Random(7)
    todo = []
    for v in VOICES:
        d = a.out / re.sub(r"[^a-z0-9]+", "_", v.lower())
        d.mkdir(parents=True, exist_ok=True)

        def add(kind, label, text, rate, pbas):
            h = hashlib.sha1(f"{v}|{text}|{rate}|{pbas}".encode()).hexdigest()[:12]
            todo.append({"voice": v, "split": split(v), "kind": kind, "label": label, "text": text,
                         "rate": rate, "pbas": pbas, "path": str(d / f"{h}.wav")})
        for w in V.LABELS[:V.WORDS]:
            for rate in (140, 180, 220):
                add("word", w, rng.choice(V.SPOKEN[w]).rstrip("?"), rate, rng.choice([None, 35, 60]))
        for t in V.CONFUSABLE:
            add("conf", "_unknown_", t, rng.choice((150, 200)), None)
        for t in V.SENTENCES[:3]:
            add("sentence", "_unknown_", t, 180, None)
    with Pool(10) as p:
        done = list(p.imap_unordered(run, todo, chunksize=8))
    (a.out / "manifest.jsonl").write_text("".join(json.dumps(j) + "\n" for j in done))
    print(f"{len(done)} say clips -> {a.out}")


if __name__ == "__main__":
    main()
