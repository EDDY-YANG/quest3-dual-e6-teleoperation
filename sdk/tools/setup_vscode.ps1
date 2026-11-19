# Prepare the editable SDK before VS Code's Python extension chooses its interpreter.
param([switch]$OpenEditor)
$ErrorActionPreference = 'Stop'
# Windows PowerShell can inherit a PowerShell 7 module path from its parent terminal.
# Use this engine's utility module for the SDK's vendor-file hash checks.
Import-Module ([IO.Path]::Combine($PSHOME, 'Modules', 'Microsoft.PowerShell.Utility', 'Microsoft.PowerShell.Utility.psd1')) -ErrorAction Stop
$sdkRoot = Split-Path -Parent $PSScriptRoot
$workspaceFile = Join-Path $sdkRoot 'Quest_E6_SDK.code-workspace'
$probe = "import sys, sysconfig; print(sys.executable); sys.exit(0 if sys.version_info[:2] == (3, 10) and sysconfig.get_platform() == 'win-amd64' else 1)"

function Find-SdkPython {
    foreach ($name in 'py.exe', 'python.exe') {
        $app = Get-Command $name -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $app) { continue }
        $probeArgs = @('-c', $probe)
        if ($name -eq 'py.exe') { $probeArgs = @('-3.10') + $probeArgs }
        try {
            $path = & $app.Source @probeArgs 2>$null
            if ($LASTEXITCODE -eq 0 -and $path -and (Test-Path -LiteralPath $path -PathType Leaf)) {
                return $path
            }
        } catch {
            # A launcher may exist without the requested interpreter. Try PATH next.
        }
    }
    throw 'Install Windows x64 Python 3.10 (reference build 3.10.11), then run this launcher again.'
}

$python = Find-SdkPython
$venvPython = Join-Path $sdkRoot 'app\.venv\Scripts\python.exe'
if (Test-Path -LiteralPath $venvPython) {
    & $venvPython -c $probe | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw 'The existing app/.venv uses a different Python. Extract the SDK into a fresh folder.'
    }
}

# The original sdk.ps1 invokes python by name. Select the verified one for this process.
$previousPath = $env:Path
try {
    $env:Path = (Split-Path -Parent $python) + [IO.Path]::PathSeparator + $previousPath
    & (Join-Path $sdkRoot 'sdk.ps1') setup
} finally {
    $env:Path = $previousPath
}

Write-Host "SDK interpreter: $venvPython"
if ($OpenEditor) {
    $code = Get-Command code.cmd -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($code) {
        & $code.Source --new-window $workspaceFile
        if ($LASTEXITCODE -ne 0) { throw 'VS Code could not open the workspace.' }
    } else {
        $candidates = @( (Join-Path $env:LOCALAPPDATA 'Programs\Microsoft VS Code\Code.exe'),
                         (Join-Path $env:ProgramFiles 'Microsoft VS Code\Code.exe') )
        $codeExe = $candidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
        if ($codeExe) {
            Start-Process -FilePath $codeExe -ArgumentList @('--new-window', ('"' + $workspaceFile + '"'))
        } else {
            throw "SDK setup passed. Install desktop VS Code or choose File > Open Workspace from File and open $workspaceFile."
        }
    }
}
