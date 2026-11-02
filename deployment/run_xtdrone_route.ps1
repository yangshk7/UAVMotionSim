param([string]$Scenario = '', [int]$TruthRateHz = 1000)
$ErrorActionPreference = 'Stop'
if ($TruthRateHz -lt 0 -or $TruthRateHz -gt 1000) { throw 'TruthRateHz must be between 0 and 1000' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$arguments = @('deployment/xtdrone/run_validation.sh', '--route')
if ($TruthRateHz -gt 0) { $arguments += @('--truth-rate-hz', [string]$TruthRateHz) }
if ($Scenario) {
    if ([IO.Path]::IsPathRooted($Scenario) -or -not (Test-Path (Join-Path $projectRoot $Scenario))) { throw 'Scenario must be an existing project-relative file' }
    $arguments += @('--scenario', $Scenario)
}
& wsl --distribution Ubuntu-24.04 --user root --cd $projectRoot --exec bash @arguments
$validationCode = $LASTEXITCODE
if ($validationCode -eq 0) {
    $latest = Get-ChildItem (Join-Path $projectRoot 'results/xtdrone_route') -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    Push-Location $projectRoot
    try {
        & uv run --no-sync python deployment/compare_gazebo_nav.py $latest.FullName
        if ($LASTEXITCODE -ne 0) { throw 'PX4 route truth comparison failed' }
        & uv run --no-sync python deployment/plot_flight.py --input $latest.FullName
        if ($LASTEXITCODE -ne 0) { throw 'PX4 route visualization failed' }
    } finally { Pop-Location }
}
exit $validationCode
