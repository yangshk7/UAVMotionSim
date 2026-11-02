$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
# Avoid round-tripping a Chinese path through PowerShell 5.1 stdout decoding.
& wsl --distribution Ubuntu-24.04 --user root --cd $projectRoot --exec bash deployment/xtdrone/run_validation.sh
$validationCode = $LASTEXITCODE
if ($validationCode -eq 0) {
    $latest = Get-ChildItem (Join-Path $projectRoot 'results/xtdrone') -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    Push-Location $projectRoot
    try {
        & uv run --no-sync python deployment/plot_flight.py --input $latest.FullName
        if ($LASTEXITCODE -ne 0) { throw 'XTDrone2 visualization failed' }
    } finally { Pop-Location }
}
exit $validationCode
