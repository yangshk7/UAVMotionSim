@echo off
setlocal
cd /d "%~dp0.."
if not exist ".venv\Scripts\pythonw.exe" (
  echo Project environment not found. Run scripts\setup.ps1 first.
  pause
  exit /b 1
)
start "" ".venv\Scripts\pythonw.exe" "main\internal\control_panel.py"
