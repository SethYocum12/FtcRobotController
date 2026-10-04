param([ValidateSet('Start','Verify','Robot')][string]$Action = 'Start')
$ErrorActionPreference = 'Stop'
$labRoot = Split-Path $PSScriptRoot -Parent
$jdk = 'C:\Program Files\Java\jdk-17'
if (!(Test-Path "$jdk\bin\java.exe")) { throw 'Install JDK 17, then update $jdk in tools/Lab.ps1.' }
$env:JAVA_HOME = $jdk
$env:GRADLE_USER_HOME = Join-Path $labRoot '.gradle-user'
$project = $labRoot
$tasks = @('installDist')
if ($Action -eq 'Verify') { $tasks = @('verifySimulation') }
if ($Action -eq 'Robot') { $project = Join-Path $labRoot 'robot-build'; $tasks = @(':TeamCode:assembleDebug') }
# Use an already installed Gradle distribution when present. Otherwise use the pinned wrapper.
$cachedGradle = Join-Path $env:USERPROFILE '.gradle\wrapper\dists\gradle-9.1.0-bin\9agqghryom9wkf8r80qlhnts3\gradle-9.1.0\bin\gradle.bat'
$gradle = Join-Path $labRoot 'gradlew.bat'
if (Test-Path $cachedGradle) { $gradle = $cachedGradle }
& $gradle -p $project -g $env:GRADLE_USER_HOME @tasks --console=plain
if ($LASTEXITCODE -ne 0) { throw "Build failed ($LASTEXITCODE). Read the output above." }
if ($Action -eq 'Start') {
    $classpath = "$labRoot\build\classes\java\main;$labRoot\build\resources\main;$labRoot\build\install\MimirSimulatorLab\lib\*"
    $app = Start-Process -FilePath "$jdk\bin\javaw.exe" -WorkingDirectory $labRoot -ArgumentList @('-cp', ('"' + $classpath + '"'), 'lab.Launcher') -PassThru
    Start-Sleep -Seconds 3
    $app.Refresh()
    if ($app.HasExited) { throw "Simulator exited during startup. Run Verify-Lab.cmd for diagnostics." }
    Write-Host "Simulator launched (process $($app.Id)). Select a Learn OpMode, then INIT and START."
}

