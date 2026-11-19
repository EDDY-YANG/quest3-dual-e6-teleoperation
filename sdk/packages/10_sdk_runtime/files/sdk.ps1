param(
    [ValidateSet('help','setup','doctor','start','shutdown','status','quest','monitor','headset',
        'lan-config','lan-info','lan-enable','lan-renew','connect-usb','connect-feedback',
        'disconnect-feedback','check-robot','prepare','arm','stop','capture-diagnostic','pack')]
    [string]$Command = 'help',
    [ValidateSet('left','right','both','auto')][string]$Side = 'auto',
    [string]$Ip = '',
    [ValidateRange(1,32)][int]$Prefix = 24,
    [string]$Profile = 'profiles\rangedik_world_anchor.json',
    [switch]$NoBrowser,
    [switch]$IncludeUsb
)
$ErrorActionPreference = 'Stop'
$LASTEXITCODE = 0
$projectRoot = $PSScriptRoot
$pythonExe = Join-Path $projectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) { $pythonExe = 'python' }
Push-Location -LiteralPath $projectRoot
try {
    switch ($Command) {
        'help' {
            Write-Host 'Read START_HERE.md. Main sequence: doctor -> start -> lan-config -> lan-enable -> connect-feedback -> prepare -> headset calibration -> arm -> stop.'
            Write-Host 'arm permits real following; stop requests robot Stop; shutdown also stops the PC service.'
            & $pythonExe -X utf8 -m quest_e6_sdk --help
        }
        'setup' {
            # LAN-first setup: no global package, PATH, network or driver changes.
            $manifest = Get-Content -LiteralPath 'vendor_manifest.json' -Raw -Encoding UTF8 | ConvertFrom-Json
            foreach ($entry in $manifest.files.PSObject.Properties) {
                if ((Get-FileHash -LiteralPath $entry.Name).Hash.ToLowerInvariant() -ne $entry.Value) {
                    throw "Pinned vendor file mismatch: $($entry.Name)"
                }
            }
            $pythonExe = Join-Path $projectRoot '.venv\Scripts\python.exe'
            if (-not (Test-Path -LiteralPath $pythonExe)) {
                & python -m venv (Join-Path $projectRoot '.venv')
                if ($LASTEXITCODE -ne 0) { throw 'venv creation failed' }
            }
            & $pythonExe -m pip install --disable-pip-version-check -r requirements-sdk.lock.txt
            if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed' }
            if ($IncludeUsb) { & (Join-Path $projectRoot '2026-10-29-14-35-setup.ps1') }
            & $pythonExe -X utf8 -m quest_e6_sdk doctor
        }
        'start' { & (Join-Path $projectRoot 'start-rangedik.ps1') -Profile $Profile -NoBrowser:$NoBrowser }
        'shutdown' { & (Join-Path $projectRoot '2026-10-29-14-35-stop-readonly.ps1') }
        'lan-config' {
            if (-not $Ip) { throw 'Pass -Ip with the current WLAN IPv4 and -Prefix with its real subnet prefix.' }
            & $pythonExe -X utf8 '2026-10-29-prepare-lan.py' --ip $Ip --prefix $Prefix
        }
        'pack' { throw 'Use the top-level sdk.ps1 pack to export the reviewed distribution.' }
        default {
            $sdkArgs = @('-X','utf8','-m','quest_e6_sdk',$Command)
            if ($Command -in @('prepare','arm')) { $sdkArgs += @('--side',$Side) }
            if ($Command -in @('connect-feedback','disconnect-feedback','check-robot')) {
                $feedbackSide = if ($Side -eq 'auto') { 'both' } else { $Side }
                $sdkArgs += @('--side',$feedbackSide)
            }
            & $pythonExe @sdkArgs
        }
    }
    if ($LASTEXITCODE -ne 0 -and $null -ne $LASTEXITCODE) { throw "SDK command failed ($LASTEXITCODE): $Command" }
} finally { Pop-Location }
