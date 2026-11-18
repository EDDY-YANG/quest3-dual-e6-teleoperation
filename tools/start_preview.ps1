$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$url = 'http://127.0.0.1:8266/'
$listener = Get-NetTCPConnection -LocalPort 8266 -State Listen -ErrorAction SilentlyContinue
if ($listener) {
    $response = Invoke-WebRequest -UseBasicParsing -Uri $url -TimeoutSec 3
    if ($response.Content -notmatch 'Quest 3 / Dual E6') {
        throw 'Port 8266 belongs to another application. No process was stopped.'
    }
    Write-Host "Preview is already running: $url"
    return
}
$pythonExe = (& python -c 'import sys; print(sys.executable)').Trim()
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $pythonExe)) {
    throw 'Python could not be located.'
}
$runtime = Join-Path $root 'runtime'
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
$stamp = [TimeZoneInfo]::ConvertTimeBySystemTimeZoneId([datetime]::UtcNow, 'China Standard Time').ToString('yyyy-MM-dd-HH-mm-ss')
$script = Join-Path $PSScriptRoot 'serve_preview.py'
$process = Start-Process -FilePath $pythonExe -ArgumentList @('-u', ('"'+$script+'"'), '--port', '8266') -WorkingDirectory $root -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $runtime "$stamp-preview-stdout.log") -RedirectStandardError (Join-Path $runtime "$stamp-preview-stderr.log")
$ready = $false
for ($attempt=0; $attempt -lt 20; $attempt++) {
    try {
        $response = Invoke-WebRequest -UseBasicParsing -Uri $url -TimeoutSec 1
        if ($response.StatusCode -eq 200 -and $response.Content -match 'Quest 3 / Dual E6') {
            $ready = $true
            break
        }
    } catch { }
    if ($process.HasExited) { throw 'Preview process exited; inspect runtime logs.' }
    Start-Sleep -Milliseconds 200
}
if (-not $ready) { throw 'Preview did not become ready; inspect runtime logs.' }
@{pid=$process.Id;url=$url;script=$script;started_at_cst=$stamp} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runtime 'preview-process.json') -Encoding UTF8
Write-Host "Preview is running in the background: $url"
Write-Host 'Closing this launcher window does not stop the preview. No startup task was installed.'
