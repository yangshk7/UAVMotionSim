$ErrorActionPreference='Stop'
$projectRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$guiPython=Join-Path $projectRoot '.venv\Scripts\pythonw.exe'
if (-not (Test-Path -LiteralPath $guiPython)) { throw 'Run scripts/setup.ps1 first.' }
# 用户请求可见的桌面上位机，直接显示 GUI；计算子进程由程序隐藏运行。
Start-Process -FilePath $guiPython -ArgumentList @('main/internal/control_panel.py') -WorkingDirectory $projectRoot -WindowStyle Normal
