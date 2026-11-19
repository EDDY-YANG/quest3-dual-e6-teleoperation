# Optional: build your own library from the included patched sources.
# Run in an x64 Visual Studio Developer shell with Rust and the Windows SDK.
param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference = 'Stop'
$appRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$source = Join-Path $appRoot 'vendor\rangedik-source-snapshot'
$out = [IO.Path]::GetFullPath($OutputDirectory)
if (Test-Path -LiteralPath $out) { throw 'Choose a new output directory.' }
New-Item -ItemType Directory -Path $out -ErrorAction Stop | Out-Null
$previousTarget = $env:CARGO_TARGET_DIR
$previousFlags = $env:CARGO_ENCODED_RUSTFLAGS
try {
    $env:CARGO_TARGET_DIR = Join-Path $out 'target'
    $cargoHome = if ($env:CARGO_HOME) { $env:CARGO_HOME } else { Join-Path $env:USERPROFILE '.cargo' }
    $env:CARGO_ENCODED_RUSTFLAGS = @("--remap-path-prefix=$env:USERPROFILE=/build-user",
        "--remap-path-prefix=$source=/rangedik","--remap-path-prefix=$cargoHome=/cargo",
        "--remap-path-prefix=$out=/build",'-C','link-arg=/PDBALTPATH:%_PDB%') -join [char]31
    & cargo build --manifest-path (Join-Path $source 'Cargo.toml') --locked --release --lib --target x86_64-pc-windows-msvc
    if ($LASTEXITCODE -ne 0) { throw 'Cargo build failed.' }
    $built = Join-Path $env:CARGO_TARGET_DIR 'x86_64-pc-windows-msvc\release\relaxed_ik_lib.dll'
    Copy-Item -LiteralPath $built -Destination $out
    Write-Host "Developer build: $out\relaxed_ik_lib.dll. The released DLL and its manifest were kept."
} finally {
    $env:CARGO_TARGET_DIR = $previousTarget
    $env:CARGO_ENCODED_RUSTFLAGS = $previousFlags
}
