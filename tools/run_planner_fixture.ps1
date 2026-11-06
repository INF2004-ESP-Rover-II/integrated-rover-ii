[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$plannerRoot = Split-Path -Parent $PSScriptRoot
$plannerCompiler = 'C:\MinGW\bin\gcc.exe'
if (-not (Test-Path -LiteralPath $plannerCompiler)) { $plannerCompiler = (Get-Command gcc -ErrorAction Stop).Source }
$plannerBuild = Join-Path $plannerRoot 'build'
New-Item -ItemType Directory -Path $plannerBuild -Force | Out-Null
$plannerExe = Join-Path $plannerBuild 'test_planner_fixture.exe'
$plannerArgs = @('-std=c11','-Wall','-Wextra','-Wconversion','-Werror','-O1',
    ('-I' + (Join-Path $plannerRoot 'firmware/robo_pico/include')),
    ('-I' + (Join-Path $plannerRoot 'firmware/robo_pico/test/fixtures')),
    (Join-Path $plannerRoot 'firmware/robo_pico/src/rover_map.c'),
    (Join-Path $plannerRoot 'firmware/robo_pico/test/fixtures/planner_fixture.c'),
    (Join-Path $plannerRoot 'firmware/robo_pico/test/test_planner_fixture.c'),
    '-o',$plannerExe)
& $plannerCompiler @plannerArgs
if ($LASTEXITCODE -ne 0) { throw 'Planner fixture build failed.' }
& $plannerExe
if ($LASTEXITCODE -ne 0) { throw 'Planner fixture checks failed.' }
Write-Host 'Offline only: no DFS, camera, sensors, MQTT or motor code is linked or run.'
