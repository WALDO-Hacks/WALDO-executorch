# Keyword spotting: MicroNet-M

WALDO's voice commands: **down, go, left, no, off, on, right, stop, up, yes** (plus
`_silence_` and `_unknown_`), from the two on-board PDM microphones.

| File | What |
|---|---|
| `kws_micronet_m.tflite` | Arm's trained MicroNet-M ([ML-Zoo](https://github.com/ARM-software/ML-zoo/tree/9f506fe52b39df545f0e6c5ff9223f671bc5ae00/models/keyword_spotting/micronet_medium), Apache-2.0), int8 TFLite, trained on Google Speech Commands |
| `micronet_m_weights.npz` | its weights, dequantised to float (`tflite_to_npz.py`) |
| `micronet.py` | the same network rebuilt in PyTorch |
| `mfcc.py` | the MFCC front end, the maths of Alif's ML Embedded Evaluation Kit (`src/waldo/kws_mfcc.c` on the board) |
| `export_kws.py` | int8 export for the Ethos-U85 with the solution's Vela settings: writes `src/kws_pte.c/.h` and the boot self-test data `src/test_kws.h` |
| `*.wav` | Speech Commands v0.02 clips (CC BY 4.0), via the Alif ML Embedded Evaluation Kit: calibration and test data |

```sh
# once, in a venv with ai-edge-litert and tflite (not the export venv):
python tflite_to_npz.py kws_micronet_m.tflite micronet_m_weights.npz
# export (needs a cbuild-mlops.yml from `cbuild setup`, like create_ai_layer.py):
.venv/bin/python model/kws/export_kws.py cmsis-executorch.cbuild-mlops.yml
```

Export result: one NPU delegate, 42 of 42 operators on the Ethos-U85, 170 KB program,
52 K MACs per 1 s window. On the sample clips the int8 model agrees with the float one,
and the float one with Arm's TFLite original.
