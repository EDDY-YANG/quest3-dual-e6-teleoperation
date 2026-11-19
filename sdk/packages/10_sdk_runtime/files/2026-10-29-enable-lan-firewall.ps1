$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$cfg = Get-Content -LiteralPath (Join-Path $root 'runtime\lan-config.json') -Raw | ConvertFrom-Json
$py = (Get-Item -LiteralPath (Join-Path $root '.venv\Scripts\python.exe')).FullName
$listener = Get-NetTCPConnection -LocalAddress $cfg.bind_ip -LocalPort 8443 -State Listen -ErrorAction SilentlyContinue
if ($listener) {
    $server = Get-CimInstance Win32_Process -Filter "ProcessId=$($listener.OwningProcess)"
    if (-not $server.CommandLine.Contains((Join-Path $root 'server.py'))) { throw 'Port 8443 is not owned by this observer.' }
    $py = $server.ExecutablePath
}
$ruleName = 'Quest3-E6-ReadOnly-HTTPS-8443'
$existing = Get-NetFirewallRule -Name $ruleName -ErrorAction SilentlyContinue
if ($existing) { Remove-NetFirewallRule -Name $ruleName }
New-NetFirewallRule -Name $ruleName -DisplayName 'Quest input only - local HTTPS 8443' -Direction Inbound -Action Allow -Protocol TCP -LocalAddress $cfg.bind_ip -LocalPort 8443 -RemoteAddress $cfg.subnet -Program $py -Profile Any | Out-Null
@{rule=$ruleName;local_ip=$cfg.bind_ip;port=8443;remote_subnet=$cfg.subnet;program=$py;success=$true} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $root 'runtime\lan-firewall-receipt.json') -Encoding UTF8
