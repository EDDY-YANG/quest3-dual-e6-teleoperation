$ErrorActionPreference = 'Stop'
$stateFile = Join-Path $PSScriptRoot 'runtime\server-process.json'
if (-not (Test-Path -LiteralPath $stateFile)) { Write-Host 'No observer process receipt.'; exit 0 }
$state = Get-Content -LiteralPath $stateFile -Raw -Encoding UTF8 | ConvertFrom-Json
$running = Get-CimInstance Win32_Process -Filter "ProcessId=$($state.pid)"
if (-not $running) { Write-Host 'Observer is already stopped.'; exit 0 }
$expectedServer = Join-Path $PSScriptRoot 'server.py'
if (-not $running.CommandLine -or -not $running.CommandLine.Contains($expectedServer)) { throw 'PID no longer belongs to this observer. Refusing to stop it.' }
$current = Invoke-RestMethod -Uri 'http://127.0.0.1:8765/api/status' -TimeoutSec 3
if ($current.project_id -ne 'quest3-dual-e6-readonly-20261029') { throw 'The active listener is not this project.' }
$trial = if ($current.teleop) { $current.teleop } else { $current.left_trial }
if ($trial.armed -or $trial.state -in @('ARMING','STOPPING','FOLLOWING','READY','WAIT_RELEASE') -or ($trial.state -eq 'FAULT' -and -not $trial.stop_evidence.feedback_confirmed) -or $current.start_positions.busy -or $current.start_positions.fault_latched) {
    $stopped = Invoke-RestMethod -Method Post -Uri 'http://127.0.0.1:8765/api/left-trial/stop' -TimeoutSec 45
    if ($stopped.armed -or ($trial.state -ne 'NOT_PREPARED' -and -not $stopped.stop_evidence.feedback_confirmed) -or $stopped.start_positions.busy -or $stopped.start_positions.fault_latched) { throw 'Robot stop was not confirmed. The service was not terminated.' }
}
Stop-Process -Id $state.pid
$listeners = Get-NetTCPConnection -LocalPort 8765 -State Listen -ErrorAction SilentlyContinue
foreach ($listener in $listeners) {
    $child = Get-CimInstance Win32_Process -Filter "ProcessId=$($listener.OwningProcess)"
    if ($child.CommandLine -and $child.CommandLine.Contains($expectedServer)) { Stop-Process -Id $child.ProcessId }
}
Write-Host 'Console stopped. Any active trial was explicitly stopped and feedback-confirmed first.'

