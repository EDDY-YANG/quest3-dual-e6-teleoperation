$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$sdkManifest = Get-Content -LiteralPath (Join-Path $projectRoot 'vendor_manifest.json') -Raw -Encoding UTF8 | ConvertFrom-Json
foreach ($entry in $sdkManifest.files.PSObject.Properties) {
    $sdkFile = Join-Path $projectRoot $entry.Name
    if (-not (Test-Path -LiteralPath $sdkFile)) { throw "Pinned DOBOT source missing: $($entry.Name). Use the complete source bundle." }
    if ((Get-FileHash -LiteralPath $sdkFile -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.Value) { throw "Pinned DOBOT source hash mismatch: $($entry.Name)" }
}
$pythonExe = Join-Path $projectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) {
    python -m venv (Join-Path $projectRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Python virtual environment creation failed.' }
}
& $pythonExe -m pip install --disable-pip-version-check -r (Join-Path $projectRoot 'requirements.lock.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
$vendorDir = Join-Path $projectRoot 'vendor'
New-Item -ItemType Directory -Path $vendorDir -Force | Out-Null
$receipt = Get-Content -LiteralPath (Join-Path $projectRoot '2026-10-29-14-35-platform-tools-receipt.json') -Raw | ConvertFrom-Json
$archivePath = Join-Path $vendorDir '2026-10-29-14-35-platform-tools-windows.zip'
if (-not (Test-Path -LiteralPath $archivePath)) { Invoke-WebRequest -Uri $receipt.source_url -OutFile $archivePath -TimeoutSec 60 }
$actualHash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualHash -ne $receipt.sha256) { throw 'Android download differs from the pinned receipt. Review the new version instead of installing it silently.' }
if (-not (Test-Path -LiteralPath (Join-Path $vendorDir 'platform-tools\adb.exe'))) { Expand-Archive -LiteralPath $archivePath -DestinationPath $vendorDir }
& (Join-Path $vendorDir 'platform-tools\adb.exe') version
Write-Host 'Ready. Run the start-readonly script. Quest Developer Mode, USB authorization and driver binding require the real headset.'

