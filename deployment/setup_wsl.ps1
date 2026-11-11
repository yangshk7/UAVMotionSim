param([string]$Distribution='Ubuntu-24.04',[switch]$CheckOnly)
$ErrorActionPreference='Stop'
$project = Split-Path -Parent $PSScriptRoot
$arguments = @('-d',$Distribution,'-u','root','--cd',$project,'--exec','bash','deployment/setup_wsl.sh')
if ($CheckOnly) { $arguments += '--check' }
& wsl @arguments
exit $LASTEXITCODE
