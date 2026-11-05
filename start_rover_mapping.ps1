[CmdletBinding()]
param(
    [string]$CameraSource = 'http://192.168.50.5/stream',
    [ValidateRange(1, 1000)][double]$MarkerMm = 74,
    [switch]$ConnectWifi,
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
$roverRoot = $PSScriptRoot
$roverBroker = '192.168.50.11'
$roverPython = Join-Path $roverRoot '.venv\Scripts\python.exe'
$roverPublisher = Join-Path $roverRoot 'vision\vision_publisher.py'
$roverDashboard = Join-Path $roverRoot 'dashboard\robot_dashboard.py'
$roverBrokerConfig = Join-Path $roverRoot 'mosquitto-rover.conf'
$roverBrokerExe = 'C:\Program Files\mosquitto\mosquitto.exe'

foreach ($roverRequired in @($roverPython, $roverPublisher, $roverDashboard, $roverBrokerConfig, $roverBrokerExe)) {
    if (-not (Test-Path -LiteralPath $roverRequired -PathType Leaf)) {
        throw "Required file is missing: $roverRequired"
    }
}

Write-Host "Camera: $CameraSource; marker width: $MarkerMm mm"
Write-Host "All rover clients use MQTT ${roverBroker}:1883; camera topic: camera/marker"
if ($CheckOnly) {
    Write-Host "Publisher: $roverPython $roverPublisher --source $CameraSource --marker-mm $MarkerMm --mqtt $roverBroker"
    Write-Host "Dashboard: $roverPython $roverDashboard (click Connect using $roverBroker)"
    Write-Host 'CheckOnly: required files exist; no network change or processes started.'
    return
}

if ($ConnectWifi) {
    netsh wlan connect name='RoverII' ssid='RoverII'
    if ($LASTEXITCODE -ne 0) { throw 'Could not request the saved RoverII Wi-Fi connection.' }
    $roverWifiDeadline = (Get-Date).AddSeconds(15)
    do {
        Start-Sleep -Milliseconds 500
        $roverOnNetwork = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object { $_.IPAddress -eq $roverBroker }).Count -gt 0
    } until ($roverOnNetwork -or (Get-Date) -ge $roverWifiDeadline)
}

$roverOnNetwork = @(Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -eq $roverBroker }).Count -gt 0
if (-not $roverOnNetwork) {
    throw "Join RoverII Wi-Fi so this laptop has IP $roverBroker, then run again. Use -ConnectWifi to request the saved profile."
}

# A separate Mosquitto service may own 127.0.0.1:1883. Use the LAN address for
# every rover client; leave unrelated localhost services alone.
$roverListeners = @(Get-NetTCPConnection -State Listen -LocalPort 1883 -ErrorAction SilentlyContinue |
    Where-Object { $_.LocalAddress -in @($roverBroker, '0.0.0.0', '::') })
$roverLogDirectory = Join-Path $roverRoot 'build\mapping-logs'
New-Item -ItemType Directory -Path $roverLogDirectory -Force | Out-Null
$roverStamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
if ($roverListeners.Count -eq 0) {
    $roverBrokerProcess = Start-Process -FilePath $roverBrokerExe `
        -ArgumentList @('-c', ('"{0}"' -f $roverBrokerConfig), '-v') `
        -WorkingDirectory $roverRoot -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $roverLogDirectory "broker-$roverStamp.log") `
        -RedirectStandardError (Join-Path $roverLogDirectory "broker-$roverStamp.err.log")
    Start-Sleep -Milliseconds 800
    $roverBrokerProcess.Refresh()
    if ($roverBrokerProcess.HasExited) { throw "Broker exited. Inspect logs in $roverLogDirectory" }
}

# Stop duplicate viewers/publishers explicitly before re-running this launcher.
$roverExisting = @(Get-CimInstance Win32_Process -Filter "Name = 'python.exe' OR Name = 'pythonw.exe'" |
    Where-Object { $_.CommandLine -match 'vision_publisher\.py|robot_dashboard\.py' })
if ($roverExisting.Count -gt 0) {
    throw 'A camera publisher or dashboard is already running. Close its window/terminal, then rerun this launcher so both use the configured broker.'
}

$roverPublisherProcess = Start-Process -FilePath $roverPython `
    -ArgumentList @('-u', ('"{0}"' -f $roverPublisher), '--source', ('"{0}"' -f $CameraSource),
        '--marker-mm', $MarkerMm.ToString([Globalization.CultureInfo]::InvariantCulture), '--mqtt', $roverBroker) `
    -WorkingDirectory $roverRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $roverLogDirectory "camera-$roverStamp.log") `
    -RedirectStandardError (Join-Path $roverLogDirectory "camera-$roverStamp.err.log")
Start-Sleep -Milliseconds 500
$roverPublisherProcess.Refresh()
if ($roverPublisherProcess.HasExited) { throw "Camera publisher exited. Inspect logs in $roverLogDirectory" }

$roverDashboardProcess = Start-Process -FilePath $roverPython `
    -ArgumentList @('-u', ('"{0}"' -f $roverDashboard)) `
    -WorkingDirectory $roverRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $roverLogDirectory "dashboard-$roverStamp.log") `
    -RedirectStandardError (Join-Path $roverLogDirectory "dashboard-$roverStamp.err.log")
Write-Host "Started camera publisher PID $($roverPublisherProcess.Id) and dashboard PID $($roverDashboardProcess.Id)."
Write-Host "In the dashboard click Connect with broker $roverBroker. Logs: $roverLogDirectory"
Write-Host 'A camera detection can be plotted once healthy robot/telemetry supplies IMU heading.'
Write-Host 'DFS exploration additionally requires starting a mission and confirming marker arrival.'
