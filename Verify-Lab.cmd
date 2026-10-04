@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\Lab.ps1" -Action Verify
if errorlevel 1 pause
pause
