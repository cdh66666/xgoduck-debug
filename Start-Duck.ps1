param([switch]$ImportOnly)
Set-StrictMode -Version 3.0
$ErrorActionPreference = 'Stop'
$taskPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $taskPython)) {
    $taskBasePython = Get-Command python -CommandType Application -ErrorAction Stop
    & $taskBasePython.Source -m venv (Join-Path $PSScriptRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Python虚拟环境创建失败，请先安装Python 3.12或3.13。' }
}
& $taskPython -m pip install --disable-pip-version-check -r (Join-Path $PSScriptRoot 'pc\requirements.txt')
if ($LASTEXITCODE -ne 0) { throw '调试台依赖安装失败。' }
$taskRequired = @('device.local.json','tools\.private\ssh-identity.json','tools\.private\id_ed25519','tools\.private\known_hosts','tools\.private\wireless-calibration-auth.json')
$taskMissing = @($taskRequired | Where-Object { -not (Test-Path -LiteralPath (Join-Path $PSScriptRoot ('pc\'+$_))) })
if ($taskMissing.Count -gt 0) {
    & $taskPython (Join-Path $PSScriptRoot 'pc\tools\import_connection.py')
    if ($LASTEXITCODE -ne 0) { throw '连接配置未导入；未启动调试台。' }
}
Remove-Item Env:\XGODUCK_CONNECTION_PASSWORD -ErrorAction SilentlyContinue
if (-not $ImportOnly) {
    Write-Host '调试台：http://127.0.0.1:8791/；启动与重连只读，不会自动上力。'
    & $taskPython -B (Join-Path $PSScriptRoot 'pc\server.py')
    if ($LASTEXITCODE -ne 0) { throw '调试台退出，请查看上面的错误。' }
}
