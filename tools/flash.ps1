# Windows version of tools/flash.sh: build for the E8 DevKit with GCC and boot it from MRAM
# through SETOOLS (no debugger).
# Board: PRG USB, SW4 on SEUART while flashing, then UART4 for the console (115200).
#   tools\flash.ps1                         build + flash
#   tools\flash.ps1 -Build                  build only
#   tools\flash.ps1 -Bin waldo.bin [-HeBin waldo_he.bin]
#                                           flash prebuilt images, no toolchain needed
# Run with: powershell -ExecutionPolicy Bypass -File tools\flash.ps1
# Needs: Alif SETOOLS for Windows ($env:ALIF_SE_TOOLS_DIR, default C:\alif\app-release-exec-windows).
# To build: CMSIS-Toolbox 2.15 on PATH, $env:GCC_TOOLCHAIN_14_3_1 = Arm GNU 14.3 bin folder.
param([switch]$Build, [string]$Bin, [string]$HeBin)
$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$SETOOLS = if ($env:ALIF_SE_TOOLS_DIR) { $env:ALIF_SE_TOOLS_DIR } else { 'C:\alif\app-release-exec-windows' }
function Run($exe) { & $exe @args; if ($LASTEXITCODE) { throw "$exe failed ($LASTEXITCODE)" } }
if ($Bin) { $Bin = (Resolve-Path $Bin).Path }
if ($HeBin) { $HeBin = (Resolve-Path $HeBin).Path }

Push-Location $Root
try {
  if ($Bin) {
    $TwoBrain = [bool]$HeBin
  } else {
    $TwoBrain = [bool](Select-String -Quiet -Pattern '^ *- WALDO_TWO_BRAIN' board\DevKit-E8\Board-U85.clayer.yml)
    Run cbuild cmsis-executorch.csolution.yml --active DevKit-E8 --toolchain GCC --packs
    if ($TwoBrain) { Run cbuild he\waldo-he.csolution.yml --active DevKit-E8-HE --packs }
    if ($Build) { return }
    $Bin = "$Root\out\cmsis-executorch\DevKit-E8\Debug\cmsis-executorch.bin"
    Run "$env:GCC_TOOLCHAIN_14_3_1\arm-none-eabi-objcopy.exe" -O binary `
      out\cmsis-executorch\DevKit-E8\Debug\cmsis-executorch.elf $Bin
    if ($TwoBrain) { $HeBin = "$Root\he\out\waldo-he\DevKit-E8-HE\Release\waldo-he.bin" }
  }
} finally { Pop-Location }

Copy-Item $Bin "$SETOOLS\build\images\waldo.bin"
if ($TwoBrain) {
  Copy-Item $HeBin "$SETOOLS\build\images\waldo_he.bin"
  $HeApp = '"HE_APP":   {"disabled": false, "binary": "waldo_he.bin", "version": "1.0.0", "signed": true,
               "cpu_id": "M55_HE", "flags": ["boot"], "mramAddress": "0x80000000"}'
  Write-Host "two-brain: HE image $((Get-Item $HeBin).Length) bytes at 0x80000000"
} else {
  Write-Host "two-brain: off (single-core build, the HE gets its stub)"
  $HeApp = '"HE_APP":   {"disabled": false, "binary": "m55_stub_he.bin", "version": "1.0.0", "signed": true,
               "loadAddress": "0x58000000", "cpu_id": "M55_HE", "flags": ["load", "boot"]}'
}
@"
{
  "DEVICE":   {"disabled": false, "binary": "app-device-config.json", "version": "0.5.00", "signed": true},
  "A32_APP":  {"disabled": false, "binary": "a32_stub_0.bin", "version": "1.0.0", "signed": true,
               "loadAddress": "0x02000000", "cpu_id": "A32_0", "flags": ["load", "boot"]},
  "HP_APP":   {"disabled": false, "binary": "waldo.bin", "version": "1.0.0", "signed": true,
               "cpu_id": "M55_HP", "flags": ["boot"], "mramAddress": "0x80080000"},
  $HeApp
}
"@ | Set-Content -Encoding ascii "$SETOOLS\build\config\waldo.json"

Push-Location $SETOOLS
try {
  Run .\app-gen-toc.exe -f build\config\waldo.json
  Run .\app-write-mram.exe -p
} finally { Pop-Location }
