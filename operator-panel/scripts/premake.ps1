$ErrorActionPreference = 'Stop'

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$projRoot = Resolve-Path (Join-Path $here '..')

$srcOperator = Join-Path $projRoot '..\src\main\deploy\operator'
$srcDashboard = Join-Path $projRoot '..\src\main\deploy\dashboard'

$dstOperator = Join-Path $projRoot 'assets\operator'
$dstDashboard = Join-Path $projRoot 'assets\dashboard'

if (Test-Path $dstOperator) { Remove-Item -Recurse -Force $dstOperator }
New-Item -ItemType Directory -Force -Path $dstOperator | Out-Null
Copy-Item -Recurse -Force (Join-Path $srcOperator '*') $dstOperator

if (Test-Path $dstDashboard) { Remove-Item -Recurse -Force $dstDashboard }
New-Item -ItemType Directory -Force -Path $dstDashboard | Out-Null
Copy-Item -Recurse -Force (Join-Path $srcDashboard '*') $dstDashboard
