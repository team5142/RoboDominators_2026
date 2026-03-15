# fetch-logs.ps1
# Copies .wpilog files from the roboRIO to a timestamped local folder.
# .wpilog files are AKit replay logs — keep ALL of them, especially match logs.
# Run this after every event to pull logs off before the rio fills up.
#
# Usage (from repo root):
#   .\scripts\fetch-logs.ps1
#
# Optional: specify a custom destination root:
#   .\scripts\fetch-logs.ps1 -Destination "D:\FRC\Logs"
#
# After fetching, delete old logs from the rio manually via:
#   ssh lvuser@10.51.42.2 "rm /home/lvuser/logs/*.wpilog"
#
# Requires: OpenSSH (built into Windows 10/11) and SCP on PATH.

param(
    [string]$RioHost     = "10.51.42.2",
    [string]$RioUser     = "lvuser",
    [string]$RioLogDir   = "/home/lvuser/logs",
    [string]$Destination = "$HOME\Documents\FRC5142\Logs"
)

$ErrorActionPreference = "Continue"

$timestamp = Get-Date -Format "yyyy-MM-dd_HH-mm-ss"
$localDir  = Join-Path $Destination $timestamp
New-Item -ItemType Directory -Path $localDir -Force | Out-Null

Write-Host ""
Write-Host "=== FRC 5142 Log Fetcher ===" -ForegroundColor Cyan
Write-Host "Rio:        $RioUser@$RioHost"
Write-Host "Saving to:  $localDir"
Write-Host ""

# List all .wpilog files on the rio
Write-Host "Querying rio for .wpilog files..." -ForegroundColor Yellow
$fileListRaw = ssh "$RioUser@$RioHost" "ls $RioLogDir/*.wpilog 2>/dev/null"
if (-not $fileListRaw) {
    Write-Host "No .wpilog files found on rio at $RioLogDir" -ForegroundColor Red
    exit 1
}

$files = $fileListRaw -split "`n" | Where-Object { $_ -match '\.wpilog$' }
Write-Host "Found $($files.Count) .wpilog file(s):" -ForegroundColor Green
$files | ForEach-Object { Write-Host "  $(Split-Path $_ -Leaf)" }
Write-Host ""

# Copy each file
$copied = 0
$failed = 0
foreach ($remoteFile in $files) {
    $remoteFile = $remoteFile.Trim()
    $fileName   = Split-Path $remoteFile -Leaf
    $src        = "${RioUser}@${RioHost}:${remoteFile}"
    Write-Host "Copying $fileName ..." -NoNewline
    scp $src $localDir
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  OK" -ForegroundColor Green
        $copied++
    } else {
        Write-Host "  FAILED (exit $LASTEXITCODE)" -ForegroundColor Red
        $failed++
    }
}

Write-Host ""
Write-Host "=== Done ===" -ForegroundColor Cyan
Write-Host "Copied:  $copied file(s)  |  Failed: $failed"
Write-Host "Saved:   $localDir"
Write-Host ""

if ($copied -gt 0) {
    Write-Host "Logs are on your PC. To free rio disk space, run:" -ForegroundColor Yellow
    Write-Host "  ssh $RioUser@$RioHost `"rm $RioLogDir/*.wpilog`"" -ForegroundColor White
    Write-Host ""
    Write-Host "Do NOT delete until you have confirmed the files copied correctly." -ForegroundColor Red
}

# Show rio disk usage after copy
$freeReport = ssh "$RioUser@$RioHost" "df -h / | tail -1"
Write-Host "Rio disk: $freeReport"
Write-Host ""
