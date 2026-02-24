@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0fetch-logs.ps1" %*
