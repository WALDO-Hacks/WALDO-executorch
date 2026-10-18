# Two-brain: the M55-HE image

WALDO's single-core build runs everything on the M55-HP. This branch splits it:

| Core | Runs | NPU |
|---|---|---|
| **M55-HE** (160 MHz, this directory) | microphones, MFCC, MicroNet keyword spotting, all the time; sleeps between audio blocks | Ethos-U55-128 |
| **M55-HP** (400 MHz, the main project) | camera, YOLOv8n, orchestrator, servos, LCD; sleeps (WFI) between interrupts | Ethos-U85-256 |

The HE posts heard words to a mailbox at the top of SRAM0 (`src/waldo/brain_link.h`),
and the HP reads them (`src/waldo/brain_hp.cpp`). The HE has no console. Its `printf`
goes into the mailbox and the HP prints it as `HE| ...`.

```
SRAM0  0x02000000  A32 boot stub
       0x02010000  HP: YOLO pools, LCD frame     (HP linker stops at 0x02380000)
       0x02380000  HE: its ExecuTorch pools      (he/linker_gnu_he.ld.src)
       0x023FF000  mailbox (4 KB)
MRAM   0x80000000  HE image (512 KB slot)   0x80080000  HP image
```

## Build and flash

`tools/flash.sh` builds both images when `WALDO_TWO_BRAIN` is set in
`board/DevKit-E8/Board-U85.clayer.yml` (the default on this branch), and boots both from
one ATOC. To go back to the single-core build, comment that line out and reflash: the
HE gets the stub again and the HP listens itself.

The HE's model is the same MicroNet, compiled for the U55-128:

```sh
.venv/bin/python model/kws/export_kws.py cmsis-executorch.cbuild-mlops.yml --he   # -> he/kws_pte_he.c
```

## What the console shows

```
two-brain: M55-HE at 160 MHz reached "self-test"; keyword self-test on its Ethos-U55: yes (0.98), ...
KWS_result: PASS
HE| HE: Ethos-U55 v1.x.x, 128 MACs/cc
two-brain: HE listening, 2 heartbeats, mic rms 40/38 (0 errors), 10 windows, MFCC ... us, U55 ... us ...
HE| HE: heard stop
```

If the HE stops at a stage, the line names it: `SE services`, `SRAM0`, `NPU`, `model`,
`self-test`, `mics`. If it never starts, the HP prints `M55-HE silent`.
