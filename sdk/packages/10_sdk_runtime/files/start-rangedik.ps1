param([string]$Profile = 'profiles\rangedik_world_anchor.json', [switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$profilePath = if ([IO.Path]::IsPathRooted($Profile)) { $Profile } else { Join-Path $projectRoot $Profile }
$profilePath = (Resolve-Path -LiteralPath $profilePath).Path
$cfg = Get-Content -LiteralPath $profilePath -Raw -Encoding UTF8 | ConvertFrom-Json
$dllPath = if ([IO.Path]::IsPathRooted($cfg.native_library)) { $cfg.native_library } else {
    [IO.Path]::GetFullPath((Join-Path (Split-Path $profilePath) $cfg.native_library))
}
if (-not (Test-Path -LiteralPath $dllPath)) { throw 'RangedIK DLL missing. Run tools\rangedik_native\build_native.ps1 first.' }
$manifestPath = Join-Path (Split-Path $dllPath) 'build-manifest.json'
if (-not (Test-Path -LiteralPath $manifestPath)) { throw 'RangedIK build manifest missing; rebuild the native backend.' }
$manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
if ($manifest.dll_sha256 -ne (Get-FileHash -LiteralPath $dllPath).Hash.ToLowerInvariant()) { throw 'RangedIK DLL does not match build manifest.' }
if (Get-NetTCPConnection -LocalPort 8765 -State Listen -ErrorAction SilentlyContinue) {
    throw 'A service is already running. Have its owner stop/confirm the robot and stop that service before selecting RangedIK.'
}
$previous = [Environment]::GetEnvironmentVariable('E6_RANGEDIK_PROFILE', 'Process')
try {
    $env:E6_RANGEDIK_PROFILE = $profilePath
    & (Join-Path $projectRoot '2026-10-29-14-35-start-readonly.ps1') -NoBrowser:$NoBrowser
} finally {
    [Environment]::SetEnvironmentVariable('E6_RANGEDIK_PROFILE', $previous, 'Process')
}
Write-Host "RangedIK output mode: $($cfg.output_mode). Startup does not connect to, enable, or move a robot."
Write-Host 'Use the existing LAN/pairing and Prepare/Arm/Grip/Stop workflow. Only configured and explicitly prepared arms can participate.'
