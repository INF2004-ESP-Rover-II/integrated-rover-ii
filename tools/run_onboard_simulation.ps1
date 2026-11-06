[CmdletBinding()]
param([switch]$NoGui,
      [ValidateRange(1,64)][int]$Columns = 8,
      [ValidateRange(1,64)][int]$Rows = 8)
$ErrorActionPreference = 'Stop'
if ($Columns * $Rows -gt 64) { throw 'The simulated arena must contain at most 64 nodes.' }
$roverRoot = Split-Path -Parent $PSScriptRoot
$roverCompiler = 'C:\MinGW\bin\gcc.exe'
if (-not (Test-Path -LiteralPath $roverCompiler)) { $roverCompiler = (Get-Command gcc -ErrorAction Stop).Source }
$roverPython = Join-Path $roverRoot '.venv\Scripts\python.exe'
$roverBuild = Join-Path $roverRoot 'build'
New-Item -ItemType Directory -Path $roverBuild -Force | Out-Null
$roverExecutable = Join-Path $roverBuild 'test_onboard_dfs.exe'
$roverTrace = Join-Path $roverBuild 'onboard-simulation.jsonl'
$roverArguments = @('-std=c11', '-Wall', '-Wextra', '-Wconversion', '-Werror', '-O1',
    ('-I' + (Join-Path $roverRoot 'firmware/robo_pico/include')),
    ('-I' + (Join-Path $roverRoot 'firmware/common')),
    (Join-Path $roverRoot 'firmware/robo_pico/test/test_onboard_dfs.c'),
    (Join-Path $roverRoot 'firmware/robo_pico/src/onboard_dfs.c'),
    (Join-Path $roverRoot 'firmware/robo_pico/src/rover_map.c'),
    (Join-Path $roverRoot 'firmware/common/vision_protocol.c'), '-lm', '-o', $roverExecutable)
& $roverCompiler @roverArguments
if ($LASTEXITCODE -ne 0) { throw 'C simulation build failed.' }
& $roverExecutable
if ($LASTEXITCODE -ne 0) { throw 'Onboard DFS tests failed.' }
& $roverExecutable --demo $Columns $Rows | Out-File -LiteralPath $roverTrace -Encoding utf8
if ($LASTEXITCODE -ne 0) { throw 'Simulation replay generation failed.' }
Write-Host "Simulation passed. Map snapshots: $roverTrace"
if (-not $NoGui) {
    & $roverPython (Join-Path $roverRoot 'dashboard/onboard_dashboard.py') --replay $roverTrace
}
