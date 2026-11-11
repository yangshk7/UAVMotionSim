$ErrorActionPreference='Stop'
Set-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)
$env:UV_CACHE_DIR=Join-Path (Get-Location) '.cache'
$env:UV_PYTHON_INSTALL_DIR=Join-Path (Get-Location) '.python'
uv python install 3.11.15
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
uv sync --locked --extra dev --python 3.11.15
exit $LASTEXITCODE
