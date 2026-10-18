#!/bin/bash
# Build for the E8 DevKit with GCC and boot it from MRAM through SETOOLS (no debugger).
# Board: PRG USB, SW4 on SEUART while flashing, then UART4 for the console (115200).
#   tools/flash.sh          build + flash
#   tools/flash.sh build    build only
# Two-brain (WALDO_TWO_BRAIN in board/DevKit-E8/Board-U85.clayer.yml): the M55-HE image
# (he/, the always-on ears) is built too and boots from the HE's MRAM slot; without it the
# HE gets the usual stub. Switching between the two needs a full flash (the ATOC changes);
# `build` only builds and leaves SETOOLS alone.
# Needs: CMSIS-Toolbox 2.15 + Arm GNU 14.3 (see tools/env.sh.example), Alif SETOOLS.
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
source ${WALDO_ENV:-~/arm-tools/env.sh}
SETOOLS=${ALIF_SE_TOOLS_DIR:-~/alif/app-release-exec-macos}
cd "$ROOT"
cbuild cmsis-executorch.csolution.yml --active DevKit-E8 --toolchain GCC --packs
TWO_BRAIN=0
grep -qE '^ *- WALDO_TWO_BRAIN' board/DevKit-E8/Board-U85.clayer.yml && TWO_BRAIN=1
if [ $TWO_BRAIN = 1 ]; then
  cbuild he/waldo-he.csolution.yml --active DevKit-E8-HE --packs
fi
[ "$1" = build ] && exit 0
ELF=out/cmsis-executorch/DevKit-E8/Debug/cmsis-executorch.elf
"$GCC_TOOLCHAIN_14_3_1/arm-none-eabi-objcopy" -O binary "$ELF" "$SETOOLS/build/images/waldo.bin"
if [ $TWO_BRAIN = 1 ]; then
  cp he/out/waldo-he/DevKit-E8-HE/Release/waldo-he.bin "$SETOOLS/build/images/waldo_he.bin"
  HE_APP='"HE_APP":   {"disabled": false, "binary": "waldo_he.bin", "version": "1.0.0", "signed": true,
               "cpu_id": "M55_HE", "flags": ["boot"], "mramAddress": "0x80000000"}'
  echo "two-brain: HE image $(wc -c < he/out/waldo-he/DevKit-E8-HE/Release/waldo-he.bin) bytes at 0x80000000"
else
  echo "two-brain: off (single-core build, the HE gets its stub)"
  HE_APP='"HE_APP":   {"disabled": false, "binary": "m55_stub_he.bin", "version": "1.0.0", "signed": true,
               "loadAddress": "0x58000000", "cpu_id": "M55_HE", "flags": ["load", "boot"]}'
fi
cat > "$SETOOLS/build/config/waldo.json" <<JSON
{
  "DEVICE":   {"disabled": false, "binary": "app-device-config.json", "version": "0.5.00", "signed": true},
  "A32_APP":  {"disabled": false, "binary": "a32_stub_0.bin", "version": "1.0.0", "signed": true,
               "loadAddress": "0x02000000", "cpu_id": "A32_0", "flags": ["load", "boot"]},
  "HP_APP":   {"disabled": false, "binary": "waldo.bin", "version": "1.0.0", "signed": true,
               "cpu_id": "M55_HP", "flags": ["boot"], "mramAddress": "0x80080000"},
  $HE_APP
}
JSON
cd "$SETOOLS"
./app-gen-toc -f build/config/waldo.json
./app-write-mram -p
