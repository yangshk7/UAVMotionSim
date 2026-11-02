$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
# Let WSL resolve the Unicode working directory natively. Capturing wslpath
# output in Windows PowerShell 5.1 can decode UTF-8 using the console code page.
& wsl --distribution Ubuntu-24.04 --user root --cd $projectRoot --exec /opt/uav_motion/sitl/4.7.2/venv/bin/python deployment/sitl/validate_flight.py --binary /opt/uav_motion/sitl/4.7.2/bin/arducopter --defaults /opt/uav_motion/sitl/4.7.2/copter.parm --output results/sitl
$validationCode = $LASTEXITCODE
if ($validationCode -eq 0) {
    $latest = Get-ChildItem (Join-Path $projectRoot 'results/sitl') -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    Push-Location $projectRoot
    try {
        & uv run --no-sync python deployment/plot_flight.py --input $latest.FullName
        if ($LASTEXITCODE -ne 0) { throw 'SITL visualization failed' }
    } finally { Pop-Location }
}
exit $validationCode
