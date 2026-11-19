param(
    [string]$Command = 'help',
    [ValidateSet('left','right','both','auto')][string]$Side = 'auto',
    [string]$Ip = '', [ValidateRange(1,32)][int]$Prefix = 24,
    [string]$Profile = 'profiles\rangedik_world_anchor.json',
    [switch]$NoBrowser, [switch]$IncludeUsb
)
$ErrorActionPreference = 'Stop'
$appRoot = Join-Path $PSScriptRoot 'app'
$assembler = Join-Path $PSScriptRoot 'tools\assemble_sdk.py'
if ($Command -eq 'pack') {
    & python -X utf8 $assembler --export
    if ($LASTEXITCODE -ne 0) { throw 'Package export failed' }
    return
}
if ($Command -eq 'assemble') {
    & python -X utf8 $assembler
    if ($LASTEXITCODE -ne 0) { throw 'Assembly failed' }
    return
}
if (-not (Test-Path -LiteralPath (Join-Path $appRoot 'sdk.ps1'))) {
    & python -X utf8 $assembler
    if ($LASTEXITCODE -ne 0) { throw 'Initial assembly failed' }
} elseif ($Command -notin @('shutdown','stop','status','quest','monitor','headset','lan-info')) {
    & python -X utf8 $assembler --check
    if ($LASTEXITCODE -ne 0) { throw 'Source packages changed. Run shutdown, then assemble.' }
}
& (Join-Path $appRoot 'sdk.ps1') -Command $Command -Side $Side -Ip $Ip -Prefix $Prefix -Profile $Profile -NoBrowser:$NoBrowser -IncludeUsb:$IncludeUsb
if ($Command -eq 'shutdown') {
    # The existing shutdown verifies process identity and Stop before returning.
    $receipt = Join-Path $appRoot 'runtime\server-process.json'
    if (Test-Path -LiteralPath $receipt) {
        $saved = Get-Content -LiteralPath $receipt -Raw -Encoding UTF8 | ConvertFrom-Json
        if (-not (Get-Process -Id $saved.pid -ErrorAction SilentlyContinue)) {
            Remove-Item -LiteralPath $receipt
        }
    }
}
