@echo off
setlocal
cd /d "%~dp0.."
where uv >nul 2>nul
if errorlevel 1 (
  echo uv was not found. Install uv and run scripts\setup.ps1 first.
  pause
  exit /b 1
)
uv run --no-sync python main\run.py %*
set "run_code=%errorlevel%"
if not "%run_code%"=="0" echo Review the generated report and logs for details.
pause
exit /b %run_code%
