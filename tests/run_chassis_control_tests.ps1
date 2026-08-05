$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$binDir = Join-Path $PSScriptRoot "bin"
$exe = Join-Path $binDir "test_chassis_control_manager.exe"

New-Item -ItemType Directory -Force -Path $binDir | Out-Null

& gcc `
    -std=c11 `
    -Wall `
    -Wextra `
    -Werror `
    (Join-Path $PSScriptRoot "test_chassis_control_manager.c") `
    (Join-Path $root "Components/Controller/Src/chassis_control_manager.c") `
    (Join-Path $root "Components/Controller/Src/chassis_mit_ctrl.c") `
    (Join-Path $root "Components/Controller/Src/chassis_brake.c") `
    (Join-Path $root "Components/Controller/Src/chassis_hold_ctrl.c") `
    -I (Join-Path $root "Components/Controller/Inc") `
    -lm `
    -o $exe

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $exe
exit $LASTEXITCODE
