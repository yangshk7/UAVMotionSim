# Explicit Windows/WSL integration regression; run with Windows PowerShell 5.1.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$previousEncoding = [Console]::OutputEncoding
try {
    foreach ($codePage in @(936, 65001)) {
        [Console]::OutputEncoding = [System.Text.Encoding]::GetEncoding($codePage)
        foreach ($relativePath in @('deployment/sitl/validate_flight.py', 'deployment/xtdrone/run_validation.sh')) {
            & wsl --distribution Ubuntu-24.04 --user root --cd $projectRoot --exec /usr/bin/test -f $relativePath
            if ($LASTEXITCODE -ne 0) {
                throw "WSL file lookup failed for code page $codePage : $relativePath"
            }
        }
        Write-Output "PASS: Windows PowerShell $($PSVersionTable.PSVersion) / code page $codePage / Unicode project path"
    }
} finally {
    [Console]::OutputEncoding = $previousEncoding
}
