#!/usr/bin/env python3
"""ElevenLabs clips for KWS v2 training and testing, cached so nothing is generated twice.

    python -I model/kws/tts_eleven.py --out DIR [--max-requests N] [--max-credits C] [--threads T]

Voices: the account's own (train; the two clones Kevin_1Shot and "me" go to test) plus
voices from the shared library picked for a spread of accent, age and gender, split by
voice into train / val / test (speaker-disjoint). Each request mixes model, text form and
speed. Every clip is DIR/clips/<sha1>.pcm (16 kHz int16) with a line in DIR/manifest.jsonl.
The API key is read by el_api (environment or .env), never printed.
"""
import argparse
import hashlib
import json
import random
import sys
import threading
import urllib.error
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import el_api  # noqa: E402
import kws_vocab as V  # noqa: E402

MODELS = [("eleven_v4_turbo", 4), ("eleven_flash_v2_5", 3), ("eleven_v4", 2), ("eleven_multilingual_v2", 1)]
ACCENTS = ["american", "british", "australian", "indian", "irish", "scottish", "african", "canadian",
           "south african", "nigerian", "jamaican", "new zealand", "singaporean", "filipino"]


def pick_shared(n_target, seed):
    """A spread of English shared-library voices: (accent, gender, age) cells round-robin."""
    seen, pool = set(), []
    for accent in ACCENTS:
        for gender in ("male", "female"):
            for age in ("young", "middle_aged", "old"):
                try:
                    vs = el_api.get_json(f"/shared-voices?page_size=8&language=en&accent={accent.replace(' ', '%20')}"
                                         f"&gender={gender}&age={age}&sort=usage_character_count_1y")["voices"]
                except Exception:
                    continue
                for v in vs:
                    if v["voice_id"] in seen:
                        continue
                    seen.add(v["voice_id"])
                    pool.append({"voice_id": v["voice_id"], "name": v.get("name", "")[:40], "accent": accent,
                                 "gender": gender, "age": age, "source": "shared"})
    rng = random.Random(seed)
    cells = {}
    for v in pool:
        cells.setdefault((v["accent"], v["gender"], v["age"]), []).append(v)
    for c in cells.values():
        rng.shuffle(c)
    out = []
    while len(out) < n_target and any(cells.values()):
        for k in sorted(cells):
            if cells[k] and len(out) < n_target:
                out.append(cells[k].pop())
    return out


def plan_voices(out, seed=20261018):
    path = out / "voices_plan.json"
    if path.exists():
        return json.loads(path.read_text())
    own = el_api.get_json("/voices")["voices"]
    voices = []
    for v in own:
        name = v["name"].split(" - ")[0]
        lab = v.get("labels") or {}
        voices.append({"voice_id": v["voice_id"], "name": name, "accent": lab.get("accent", "?"),
                       "gender": lab.get("gender", "?"), "age": lab.get("age", "?"), "source": "own",
                       "split": "test" if name in ("Kevin_1Shot", "me") else "train"})
    shared = pick_shared(110, seed)
    rng = random.Random(seed)
    rng.shuffle(shared)
    # Stratified: every 3rd voice of each accent group to held-out sets
    by_acc = {}
    for v in shared:
        by_acc.setdefault(v["accent"], []).append(v)
    for acc, vs in by_acc.items():
        for i, v in enumerate(vs):
            v["split"] = "test" if i % 4 == 0 else "val" if i % 4 == 2 else "train"
    voices += shared
    path.write_text(json.dumps(voices, indent=1))
    return voices


def jobs(voices, seed=1):
    rng = random.Random(seed)
    models = [m for m, w in MODELS for _ in range(w)]
    for v in voices:
        held = v["split"] != "train"
        def job(kind, label, text):
            return {"voice_id": v["voice_id"], "voice": v["name"], "split": v["split"], "kind": kind,
                    "label": label, "text": text, "model": rng.choice(models),
                    "speed": round(rng.uniform(0.85, 1.15), 2), "stability": round(rng.uniform(0.3, 0.8), 2)}
        for w in V.LABELS[:V.WORDS]:
            n = 2 if held else (1 if w in V.COMMANDS else 4)
            for t in rng.sample(V.SPOKEN[w], n):
                yield job("word", w, t)
        for t in (V.CONFUSABLE if held else rng.sample(V.CONFUSABLE, 14)):
            yield job("conf", "_unknown_", t)
        for t in rng.sample(V.SENTENCES, 2 if held else 1):
            yield job("sentence", "_unknown_", t)


def key_of(j):
    return hashlib.sha1(json.dumps([j[k] for k in ("voice_id", "text", "model", "speed", "stability")]).encode()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max-requests", type=int, default=10000)
    ap.add_argument("--max-credits", type=float, default=30000)
    ap.add_argument("--threads", type=int, default=6)
    ap.add_argument("--plan-only", action="store_true")
    a = ap.parse_args()
    (a.out / "clips").mkdir(parents=True, exist_ok=True)
    voices = plan_voices(a.out)
    todo = list(jobs(voices))
    for s in ("train", "val", "test"):
        vs = [v for v in voices if v["split"] == s]
        print(f"{s}: {len(vs)} voices, {sum(1 for j in todo if j['split'] == s)} clips; accents "
              f"{sorted({v['accent'] for v in vs})}; genders {sorted({v['gender'] for v in vs})}")
    if a.plan_only:
        return
    man = a.out / "manifest.jsonl"
    done = set()
    if man.exists():
        done = {json.loads(l)["key"] for l in man.read_text().splitlines() if l.strip()}
    lock = threading.Lock()
    state = {"req": 0, "credits": 0.0, "stop": None, "ok": 0, "err": 0}
    fh = open(man, "a")

    def work(j):
        k = key_of(j)
        if k in done or state["stop"]:
            return
        with lock:
            if state["req"] >= a.max_requests or state["credits"] >= a.max_credits:
                state["stop"] = state["stop"] or "cap reached"
                return
            state["req"] += 1
        try:
            pcm, cost = el_api.tts(j["voice_id"], j["text"], j["model"],
                                   {"stability": j["stability"], "similarity_boost": 0.75, "speed": j["speed"]})
        except urllib.error.HTTPError as e:
            with lock:
                state["err"] += 1
                if e.code in (401, 402, 403):
                    state["stop"] = f"HTTP {e.code}"
            return
        except Exception:
            with lock:
                state["err"] += 1
            return
        (a.out / "clips" / f"{k}.pcm").write_bytes(pcm)
        with lock:
            state["credits"] += cost
            state["ok"] += 1
            fh.write(json.dumps({**j, "key": k, "cost": cost}) + "\n")
            fh.flush()
            if state["ok"] % 250 == 0:
                print(f"  {state['ok']} clips, {state['credits']:.0f} credits, {state['err']} errors", flush=True)

    with ThreadPoolExecutor(a.threads) as ex:
        list(ex.map(work, todo))
    fh.close()
    print(f"done: {state['ok']} new clips, {state['req']} requests, {state['credits']:.0f} credits, "
          f"{state['err']} errors, stop={state['stop']}")


if __name__ == "__main__":
    main()
