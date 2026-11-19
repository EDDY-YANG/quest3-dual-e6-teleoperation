@echo off
cd /d "%~dp0"
if not exist "app\.venv\Scripts\python.exe" (
 powershell -NoProfile -ExecutionPolicy Bypass -File sdk.ps1 setup
 if errorlevel 1 exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File sdk.ps1 start
pause
