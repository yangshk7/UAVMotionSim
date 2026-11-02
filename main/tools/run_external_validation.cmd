@echo off
setlocal
cd /d "%~dp0..\.."
set "review_dir="
set "sitl_code=not_run"
set "xtdrone_code=not_run"
for /f "delims=" %%R in ('uv run --no-sync python deployment/create_report.py --begin') do set "review_dir=%%R"
if not defined review_dir goto report_failed
powershell.exe -NoProfile -ExecutionPolicy Bypass -File deployment\run_sitl_validation.ps1
set "sitl_code=%errorlevel%"
if not "%sitl_code%"=="0" goto report
powershell.exe -NoProfile -ExecutionPolicy Bypass -File deployment\run_xtdrone_validation.ps1
set "xtdrone_code=%errorlevel%"
:report
uv run --no-sync python deployment/create_report.py --run-dir "%review_dir%" --sitl-code "%sitl_code%" --xtdrone-code "%xtdrone_code%"
set "run_code=%errorlevel%"
echo Review: %review_dir%/index.html
if /I not "%~1"=="--no-pause" start "" "%review_dir%\index.html"
if /I not "%~1"=="--no-pause" pause
exit /b %run_code%
:report_failed
echo Could not create the external review directory. Check uv and the project environment.
if /I not "%~1"=="--no-pause" pause
exit /b 1
