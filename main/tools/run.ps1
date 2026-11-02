param(
    [ValidateSet('auto','required','skip')][string]$Matlab='auto',
    [int]$Workers=0,
    [switch]$NoOpen
)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath ([IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..')))
$runArguments=@('run','--no-sync','python','main/internal/run.py','--matlab',$Matlab,'--workers',"$Workers")
if ($NoOpen) { $runArguments+='--no-open' }
& uv @runArguments
exit $LASTEXITCODE
