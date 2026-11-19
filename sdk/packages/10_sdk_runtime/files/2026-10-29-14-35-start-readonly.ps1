param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$pythonExe = Join-Path $projectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) { throw 'Run the setup script first.' }
$listener = Get-NetTCPConnection -LocalPort 8765 -State Listen -ErrorAction SilentlyContinue
if ($listener) {
    $existing = Invoke-RestMethod -Uri 'http://127.0.0.1:8765/api/status' -TimeoutSec 3
    if ($existing.project_id -ne 'quest3-dual-e6-readonly-20261029') { throw 'Port 8765 is used by another application.' }
    Write-Host 'The Quest/E6 console is already running.'
} else {
    $runtimeDir = Join-Path $projectRoot 'runtime'
    New-Item -ItemType Directory -Path $runtimeDir -Force | Out-Null
    $stamp = [TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([datetime]::UtcNow,'China Standard Time').ToString('yyyy-MM-dd-HH-mm-ss')
    $serverFile = Join-Path $projectRoot 'server.py'
    $process = Start-Process -FilePath $pythonExe -ArgumentList @('-X','utf8',('"' + $serverFile + '"')) -WorkingDirectory $projectRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $runtimeDir "$stamp-server-stdout.log") -RedirectStandardError (Join-Path $runtimeDir "$stamp-server-stderr.log")
    @{pid=$process.Id;server=$serverFile;started_at=$stamp} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runtimeDir 'server-process.json') -Encoding UTF8
    $ready = $false
    for ($attempt=0; $attempt -lt 30; $attempt++) {
        Start-Sleep -Milliseconds 200
        try {
            $state = Invoke-RestMethod -Uri 'http://127.0.0.1:8765/api/status' -TimeoutSec 1
            if ($state.project_id -eq 'quest3-dual-e6-readonly-20261029') { $ready=$true; break }
        } catch { }
        if ($process.HasExited) { throw 'Observer exited. Inspect the latest runtime stderr log.' }
    }
    if (-not $ready) { throw 'Observer startup did not complete. Inspect runtime logs.' }
}
Write-Host 'PC: http://127.0.0.1:8765'
Write-Host 'Quest after USB setup: http://localhost:8765/quest'
Write-Host 'No automatic robot connection at startup. Explicit Prepare/Arm uses the connected single-arm or dual-arm group.'
Write-Host 'Alarm clear, mode switching and command replay are not automatic. Service startup does not arm motion.'
if (-not $NoBrowser) { Start-Process 'http://127.0.0.1:8765' }

