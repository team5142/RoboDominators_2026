# Removes old renamed auto files from the RIO deploy directory.
# Run this once while connected to the robot's network.

$rio = "admin@10.51.42.2"
$autoDir = "/home/lvuser/deploy/pathplanner/autos"

$oldAutos = @(
    "Blue-LeftBumpToCenterToOutpost.auto",
    "Blue-RightBumpToCenterToOutpost.auto",
    "Blue-RightBumpToCenter.auto"
)

$files = ($oldAutos | ForEach-Object { "$autoDir/$_" }) -join " "

Write-Host "Connecting to RIO and removing old autos..."
ssh $rio "rm -f $files && echo 'Done - old autos removed' || echo 'Failed'"
