param([string]$Config='configs/rotorpy_hex_x12.yaml',[int]$Passes=1)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)
uv run --no-sync python -m uavsim.cli run --config $Config --passes $Passes
exit $LASTEXITCODE
