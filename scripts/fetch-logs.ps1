# fetch-logs.ps1
# Copies the last 10 log sessions from the roboRIO to a timestamped local folder,
# then deletes those sessions from the rio to free disk space.
#
# Usage (from any PowerShell):
#   .\scripts\fetch-logs.ps1
#
# Optional: specify a custom destination root:
#   .\scripts\fetch-logs.ps1 -Destination "D:\FRC\Logs"
#
# Requires: OpenSSH (built into Windows 10/11) and SCP on PATH.

param(
    [string]$RioHost    = "10.51.42.2",
    [string]$RioUser    = "lvuser",
    [string]$RioLogDir  = "/home/lvuser/logs",
    [string]$Destination = "$HOME\Documents\FRC5142\Logs",
    [int]   $SessionCount = 10
)

$ErrorActionPreference = "Continue"

# Timestamped local folder for this pull
$timestamp  = Get-Date -Format "yyyy-MM-dd_HH-mm-ss"
$localDir   = Join-Path $Destination $timestamp
New-Item -ItemType Directory -Path $localDir -Force | Out-Null

Write-Host ""
Write-Host "=== FRC 5142 Log Fetcher ===" -ForegroundColor Cyan
Write-Host "Rio:        $RioUser@$RioHost"
Write-Host "Saving to:  $localDir"
Write-Host ""

# Get the last N session folder names from the rio (newest last, take last N)
Write-Host "Querying rio for session list..." -ForegroundColor Yellow
$sessionListRaw = ssh "$RioUser@$RioHost" "ls -dt $RioLogDir/*/ 2>/dev/null | tail -n $SessionCount | xargs -I{} basename {}"
if (-not $sessionListRaw) {
    Write-Host "No log sessions found on rio." -ForegroundColor Red
    exit 1
}

$sessions = $sessionListRaw -split "`n" | Where-Object { $_ -match '\S' }
Write-Host "Found $($sessions.Count) sessions to fetch:" -ForegroundColor Green
$sessions | ForEach-Object { Write-Host "  $_" }
Write-Host ""

# Copy each session folder using scp -r
$copied  = 0
$failed  = 0
foreach ($session in $sessions) {
    $src = "${RioUser}@${RioHost}:${RioLogDir}/${session}"
    Write-Host "Copying $session ..." -NoNewline
    scp -r $src $localDir
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  OK" -ForegroundColor Green
        $copied++
    } else {
        Write-Host "  FAILED (exit $LASTEXITCODE)" -ForegroundColor Red
        $failed++
    }
}

Write-Host ""
if ($copied -eq 0) {
    Write-Host "No sessions copied. Aborting delete." -ForegroundColor Red
    exit 1
}

# Delete the copied sessions from the rio
Write-Host "Deleting $copied fetched sessions from rio..." -ForegroundColor Yellow
foreach ($session in $sessions) {
    $remotePath = "$RioLogDir/$session"
    ssh "$RioUser@$RioHost" "rm -rf '$remotePath'"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "  WARNING: Delete may have failed for $session" -ForegroundColor Yellow
    }
}

# Report free space after cleanup
$freeReport = ssh "$RioUser@$RioHost" "df -h / | tail -1"
Write-Host ""
Write-Host "=== Done ===" -ForegroundColor Cyan
Write-Host "Copied:  $copied sessions  |  Failed: $failed"
Write-Host "Saved:   $localDir"
Write-Host "Rio disk after cleanup: $freeReport"
Write-Host ""
