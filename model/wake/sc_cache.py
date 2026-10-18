#!/usr/bin/env python3
"""Index Speech Commands v0.02 (CC BY 4.0, untrusted input: only wav samples and the two
split lists are read) into int16 memmaps under <paths.cache>/sc_<split>.npy with word labels
and speaker ids, plus the background noise split by time (config augment/eval noise_split).

    python -I model/wake/sc_cache.py
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wakelib  # noqa: E402


def main():
    cfg = wakelib.load_config()
    sc = Path(cfg["paths"]["speech_commands"])
    out = Path(cfg["paths"]["cache"])
    out.mkdir(parents=True, exist_ok=True)
    listed = {s: set((sc / f"{n}_list.txt").read_text().split()) for s, n in (("val", "validation"), ("test", "testing"))}
    files = sorted(p for p in sc.glob("*/*.wav") if not p.parent.name.startswith("_"))
    split = {"train": [], "val": [], "test": []}
    for p in files:
        rel = f"{p.parent.name}/{p.name}"
        split["val" if rel in listed["val"] else "test" if rel in listed["test"] else "train"].append(p)
    meta = {}
    for s, lst in split.items():
        mm = np.lib.format.open_memmap(out / f"sc_{s}.npy", "w+", np.int16, (len(lst), wakelib.WINDOW))
        words, spk = [], []
        for i, p in enumerate(lst):
            a = wakelib.read_wav(p)[:wakelib.WINDOW]
            mm[i, :len(a)] = a
            mm[i, len(a):] = 0
            words.append(p.parent.name)
            spk.append(p.name.split("_")[0])
        mm.flush()
        meta[s] = {"words": words, "speakers": spk}
        print(s, len(lst), "clips,", len(set(spk)), "speakers")
    # speaker disjointness is a property of the official lists; verify it
    for a, b in (("train", "val"), ("train", "test"), ("val", "test")):
        assert not set(meta[a]["speakers"]) & set(meta[b]["speakers"]), f"speaker leak {a}/{b}"
    (out / "sc_meta.json").write_text(json.dumps(meta))
    fr = cfg["eval"]["noise_split"]
    noise = {"train": [], "val": [], "test": []}
    for p in sorted((sc / "_background_noise_").glob("*.wav")):
        a = wakelib.read_wav(p)
        c1, c2 = int(len(a) * fr[0]), int(len(a) * (fr[0] + fr[1]))
        noise["train"].append(a[:c1]); noise["val"].append(a[c1:c2]); noise["test"].append(a[c2:])
    for s, parts in noise.items():
        np.save(out / f"noise_{s}.npy", np.concatenate(parts))
        np.save(out / f"noise_{s}_lens.npy", np.array([len(x) for x in parts]))
        print("noise", s, sum(len(x) for x in parts) / wakelib.RATE, "s")


if __name__ == "__main__":
    main()
