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

## Flash and read the console

Same as the single-core build: SW4 on SEUART, `tools/flash.sh` (it prints `two-brain: HE
image N bytes at 0x80000000`), then SW4 on UART4, reset, console at 115200.

Which build is running: the two-brain HP prints `WALDO two-brain: YOLOv8n ... MicroNet KWS
on the M55-HE's Ethos-U55` and `WALDO: two-brain build, the M55-HE owns the microphones`;
the single-core build prints a `voice: mic rms ...` stats line instead.

A working boot:

```
two-brain: M55-HE at 160 MHz reached "self-test"; keyword self-test on its Ethos-U55: yes (0.98), ...
KWS_result: PASS
HE| HE: Ethos-U55 v1.x.x, 128 MACs/cc
YOLO_result: PASS
two-brain: HE listening, 2 heartbeats, mic rms 40/38 (0 errors), 10 windows, MFCC ... us, U55 ... us ...
HE| HE: heard stop
HEARD: stop
```

## When the HE doesn't work

The HP waits up to 5 s at boot for the HE's self-test. YOLO, the camera and the servos
work either way; only voice depends on the HE.

| Console says | Meaning | Try |
|---|---|---|
| `no word from the M55-HE after 5000 ms` + `KWS_result: FAIL`; later `M55-HE silent (no mailbox)` | the HE image never ran, or never reached SRAM0 | check `~/alif/app-release-exec-macos/build/app-package-map.txt` lists `waldo_he.bin` at 0x80000000; reflash |
| `reached "SE services"` or `"SRAM0"` and FAILED | the HE couldn't talk to the Secure Enclave | power-cycle, reflash |
| `reached "NPU"` and FAILED | the U55 driver didn't start | `HE|` log lines say why |
| `reached "model"` and FAILED | ExecuTorch couldn't load the U55 program (pools in SRAM0 at 0x02380000) | `HE|` log lines; `WALDO_HE_POOLS_DTCM` (below); re-export with `--he` |
| `self-test: wrong word` | the U55 runs but the result is wrong | compare with the single-core `KWS_result` |
| stuck at `"mics"`, or `mic rms 0/0` | the PDM didn't start on the HE | fall back |

**If the HE stops at `"model"` or `"self-test"`:** try the pools in the HE's DTCM instead
of SRAM0 (Alif's own HE examples keep the NPU arena there). Uncomment `WALDO_HE_POOLS_DTCM`
and `MAX_KERNEL_NUM` in `he/waldo-he.cproject.yml` and run `tools/flash.sh` again. The HE
log then says `KWS pools at 0x2000....` and `method pool N of 8192 B used`. It uses the
same `kws_pte_he.c`: Vela's command stream addresses the scratch through a base register,
so no re-export is needed. DTCM then holds 124 KiB of temp pool (Vela scratch: 123600 B),
an 8 KiB method pool, a 0.5 s mic ring and a 16 KiB heap, with 16 KiB to spare.

**Fallback:** comment out `- WALDO_TWO_BRAIN` in `board/DevKit-E8/Board-U85.clayer.yml`
and run `tools/flash.sh` again (a full flash: the ATOC changes). The HP listens itself, as
on branch `yolo`.

## Re-exporting the HE model

Needs the project's Python venv (`.venv`, see the main README) and
`cmsis-executorch.cbuild-mlops.yml`, which `cbuild setup` writes (it is not committed):

```sh
.venv/bin/python model/kws/export_kws.py cmsis-executorch.cbuild-mlops.yml --he   # -> he/kws_pte_he.c
```

## Harmless build noise

csolution prints `redefinition from 'AC6' into 'GCC'` and a few `update recommended ...
--update-rte` lines; ignore them. The HE builds as Release (`optimize: size`), the HP as
Debug.

## If YOLO also needs `WALDO_NPU_SRAM0_ONLY`

That layout fills SRAM0 up to 0x023F4000, which overlaps the HE's 512 KB share. Combine it
with `WALDO_HE_POOLS_DTCM` (the HE then keeps only the 4 KB mailbox in SRAM0) and set
`APP_SRAM0_HE_RESERVED` to `0x00001000` in
`board/DevKit-E8/RTE/Device/AE822FA0E5597LS0_M55_HP/app_mem_regions.h`.
