@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0fetch-logs.ps1" -RioHost 10.51.42.2 %*
