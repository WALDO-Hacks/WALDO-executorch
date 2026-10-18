"""ElevenLabs API access for the KWS data scripts. The key comes from the environment
(ELEVENLABS_API_KEY) or from the .env file named by KWS_ENV_FILE; it is never printed."""
import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path

API = "https://api.elevenlabs.io/v1"


def key():
    k = os.environ.get("ELEVENLABS_API_KEY")
    if k:
        return k
    env = Path(os.environ.get("KWS_ENV_FILE", "/Users/bentomac/baseline/.env"))
    for line in env.read_text().splitlines():
        if line.startswith("ELEVENLABS_API_KEY="):
            return line.split("=", 1)[1].strip().strip('"').strip("'")
    raise SystemExit("ELEVENLABS_API_KEY not set")


def call(path, body=None, method=None, timeout=60):
    """-> (bytes, headers). Raises urllib.error.HTTPError (the key is not in its message)."""
    req = urllib.request.Request(API + path, data=json.dumps(body).encode() if body is not None else None,
                                 headers={"xi-api-key": key(), "Content-Type": "application/json"},
                                 method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read(), dict(r.headers)


def get_json(path):
    return json.loads(call(path)[0])


def tts(voice_id, text, model, settings=None, retries=4):
    """-> (int16 pcm bytes at 16 kHz, credits charged) or raises the last HTTPError."""
    body = {"text": text, "model_id": model}
    if settings:
        body["voice_settings"] = settings
    last = None
    for attempt in range(retries):
        try:
            b, h = call(f"/text-to-speech/{voice_id}?output_format=pcm_16000", body)
            return b, float(h.get("character-cost", h.get("Character-Cost", 0)) or 0)
        except urllib.error.HTTPError as e:
            last = e
            if e.code in (401, 402, 403):
                raise
            time.sleep(1.5 + 2 * attempt)
        except Exception as e:  # network hiccup
            last = e
            time.sleep(1.5 + 2 * attempt)
    raise last
