param([string]$Scenario = '', [int]$TruthRateHz = 300)
$ErrorActionPreference = 'Stop'
if ($TruthRateHz -lt 1 -or $TruthRateHz -gt 1000) { throw 'TruthRateHz must be between 1 and 1000' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$arguments = @('deployment/sitl/validate_flight.py', '--binary', '/opt/uav_motion/sitl/4.7.2/bin/arducopter', '--defaults', '/opt/uav_motion/sitl/4.7.2/copter.parm', '--output', 'results/sitl_route', '--route')
$arguments += @('--truth-rate-hz', [string]$TruthRateHz)
if ($Scenario) {
    if ([IO.Path]::IsPathRooted($Scenario) -or -not (Test-Path (Join-Path $projectRoot $Scenario))) { throw 'Scenario must be an existing project-relative file' }
    $arguments += @('--scenario', $Scenario)
}
& wsl --distribution Ubuntu-24.04 --user root --cd $projectRoot --exec /opt/uav_motion/sitl/4.7.2/venv/bin/python @arguments
$validationCode = $LASTEXITCODE
if ($validationCode -eq 0) {
    $latest = Get-ChildItem (Join-Path $projectRoot 'results/sitl_route') -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    Push-Location $projectRoot
    try {
        & uv run --no-sync python deployment/compare_sitl_truth.py $latest.FullName
        if ($LASTEXITCODE -ne 0) { throw 'SITL truth comparison failed' }
        & uv run --no-sync python deployment/plot_flight.py --input $latest.FullName
        if ($LASTEXITCODE -ne 0) { throw 'SITL route visualization failed' }
    } finally { Pop-Location }
}
exit $validationCode
