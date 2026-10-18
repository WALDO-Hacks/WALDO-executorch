# Keyword spotting

WALDO listens with the two on-board PDM microphones: MFCCs on the Cortex-M55, the network
on the Ethos-U (the HE's U55-128 in the two-brain build, the U85 in the single-core build).

Two models, one switch (`src/kws_model.h`, for both images):

| Model | Labels | Files |
|---|---|---|
| **KWS v2** (default) | 10 commands: down go left no off on right stop up yes; 10 objects: person cup bottle phone laptop book backpack keyboard dog cat; home, patrol, hey_waldo; `_silence_`, `_unknown_` (25) | `src/kws_pte.c`, `he/kws_pte_he.c` |
| MicroNet-M (`WALDO_KWS_MICRONET`) | the 10 commands, `_silence_`, `_unknown_` (12) | `src/kws_pte_micronet.c`, `he/kws_pte_he_micronet.c` |

`src/kws_labels.h` (generated) holds both label sets, a per-word on/off switch and a
per-word threshold. The orchestrator maps the words (`src/waldo/orchestrator.cpp`): an object
word alone follows that YOLO class (and resumes from HOLD), "home" returns to SENTRY at
home, "patrol" starts the patrol at once, "hey_waldo" opens the wake-word window.

Classes left out of the vocabulary: the farm and zoo animals (never on a table), dining
table (the turret stands on it), couch, chair and TV (big and static, nothing to follow),
potted plant, clock, remote, handbag, mouse, scissors and bird (small or rare at a demo
table, weak for YOLO at our camera resolution).

## KWS v2

Same network and input as MicroNet-M (49 x 10 MFCC from `mfcc.py`, the board's front end
bit for bit): Arm's trained weights, a new 196 -> 25 classifier whose old rows start from
Arm's, fine-tuned on

- Google Speech Commands v0.02 (CC BY 4.0): the 10 commands, dog, cat, the other words as
  `_unknown_`, the background noise; its official speaker-disjoint train/val/test lists;
- ElevenLabs voices (`tts_eleven.py`): the account's voices and a spread of shared-library
  voices (accents, ages, genders), split by voice into train/val/test; the two clones of a
  team member are test only; several models, text forms and speeds;
- macOS `say` voices (`tts_say.py`), split by voice name;
- near misses ("cap", "pup", "hey Walter", ...), sentences and word fragments as `_unknown_`.

Augmentation (`kws2_data.py`) aims at the DevKit's microphones 30-150 cm away in a room:
reverb with a distance-dependent direct/reverberant ratio, mic high-pass/low-pass/resonance,
noise and babble at 3-30 dB SNR, the board's measured levels (speech 500-1200 rms over a
165-800 rms floor), gain, time shift, speed, DC offset.

| File | What |
|---|---|
| `kws_vocab.py` | labels, spoken forms, object -> YOLO class |
| `el_api.py`, `tts_eleven.py`, `tts_say.py` | TTS data (cached; key from the environment or `.env`, never printed) |
| `kws2_data.py`, `kws2_featurize.py` | corpora, augmentation, MFCC windows |
| `kws2_train.py` | fine-tuning, writes `best.npz` in the `micronet_m_weights.npz` layout |
| `kws2_eval.py` | the gate report: v2 vs MicroNet-M with the board's listening logic on held-out speakers |

```sh
K=/path/outside/the/repo   # data, caches, runs
PY=.venv/bin/python
mkdir -p $K/sc && curl -L http://download.tensorflow.org/data/speech_commands_v0.02.tar.gz | tar -xz -C $K/sc
$PY -I model/kws/tts_eleven.py --out $K/tts_el --max-requests 12000 --max-credits 40000
$PY -I model/kws/tts_say.py --out $K/tts_say
$PY -I model/kws/kws2_featurize.py $K
$PY -I model/kws/kws2_train.py $K --out $K/runs/r1
$PY -I model/kws/kws2_eval.py $K --weights $K/runs/r1/best.npz
$PY model/kws/export_kws.py cmsis-executorch.cbuild-mlops.yml --weights $K/runs/r1/best.npz --calib $K/feats/val.npz
$PY model/kws/export_kws.py cmsis-executorch.cbuild-mlops.yml --weights $K/runs/r1/best.npz --calib $K/feats/val.npz --he
```

## MicroNet-M (fallback)

| File | What |
|---|---|
| `kws_micronet_m.tflite` | Arm's trained MicroNet-M ([ML-Zoo](https://github.com/ARM-software/ML-zoo/tree/9f506fe52b39df545f0e6c5ff9223f671bc5ae00/models/keyword_spotting/micronet_medium), Apache-2.0), int8 TFLite, trained on Google Speech Commands |
| `micronet_m_weights.npz` | its weights, dequantised to float (`tflite_to_npz.py`) |
| `micronet.py` | the same network rebuilt in PyTorch |
| `mfcc.py` | the MFCC front end, the maths of Alif's ML Embedded Evaluation Kit (`src/waldo/kws_mfcc.c` on the board) |
| `export_kws.py` | int8 export with the solution's Vela settings; `--micronet` for this model |
| `*.wav` | Speech Commands v0.02 clips (CC BY 4.0), via the Alif ML Embedded Evaluation Kit: calibration and test data |

```sh
# once, in a venv with ai-edge-litert and tflite (not the export venv):
python tflite_to_npz.py kws_micronet_m.tflite micronet_m_weights.npz
.venv/bin/python model/kws/export_kws.py cmsis-executorch.cbuild-mlops.yml --micronet [--he]
```

Export result: one NPU delegate, 42 of 42 operators on the Ethos-U85, 170 KB program,
52 K MACs per 1 s window. On the sample clips the int8 model agrees with the float one,
and the float one with Arm's TFLite original.
