@echo off
setlocal
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\setup_vscode.ps1" -OpenEditor
if errorlevel 1 (
  echo.
  echo Read the message above and try again.
  pause
  exit /b 1
)
exit /b 0
