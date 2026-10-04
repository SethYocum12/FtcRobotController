@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\Lab.ps1" -Action Robot
if errorlevel 1 pause
pause
