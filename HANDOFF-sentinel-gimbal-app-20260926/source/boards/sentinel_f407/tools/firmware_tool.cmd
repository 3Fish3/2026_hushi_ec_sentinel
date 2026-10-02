@echo off
setlocal
cd /d "%~dp0..\..\.."
if not exist ".venv\Scripts\python.exe" (
  echo Project Python not found: .venv\Scripts\python.exe
  pause
  exit /b 1
)
".venv\Scripts\python.exe" "boards\sentinel_f407\tools\firmware_tool.py"
if errorlevel 1 pause
