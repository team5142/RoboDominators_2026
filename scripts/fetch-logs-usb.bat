@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0fetch-logs.ps1" -RioHost 172.22.11.2 %*
