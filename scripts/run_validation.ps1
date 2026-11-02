$ErrorActionPreference='Stop'
Set-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)
uv run --no-sync python -m uavsim.cli validate --config configs/experiment_two_pass.yaml
exit $LASTEXITCODE
