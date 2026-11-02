param(
    [ValidateSet('auto','required','skip')][string]$Matlab='auto',
    [int]$Workers=0,
    [switch]$NoOpen
)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)
$runArguments=@('run','--no-sync','python','main/run.py','--matlab',$Matlab,'--workers',"$Workers")
if ($NoOpen) { $runArguments+='--no-open' }
& uv @runArguments
exit $LASTEXITCODE
