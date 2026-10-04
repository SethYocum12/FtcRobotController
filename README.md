# Mimir simulator learning lab

This is a separate project. The original FtcRobotController project is not changed.

## Start here with Dad

1. Double-click **Start-Simulator.cmd**. It rebuilds your edits before opening the simulator.
2. Check Configuration is **MecDynamic**. Select **Learn - Left** in Op Mode.
3. Click **INIT**, then **START**. Read telemetry: target (60, 72), heading 90 degrees; Result should become Reached target. Click **STOP** before choosing another OpMode.
4. Try **Learn - Forward** (72, 84, 90 degrees), then **Learn - Turn** (72, 72, 180 degrees).

The simulator may show the robot elsewhere on its graphic: the lesson sets an odometry coordinate frame at startup. Telemetry is the coordinate reference for these exercises. No physical gamepad is required for these autonomous lessons.

## First lesson: predict, change, test, explain

Open `shared/src/org/firstinspires/ftc/teamcode/learning/LearningAuto.java`.

Before running, ask Seth: If left decreases X, what should the endpoint be when DISTANCE_INCHES changes from 12 to 18? Have him change just that number, close the simulator, and launch again. Compare his prediction against telemetry. Ask why forward changes Y at the starting heading of 90 degrees. Then restore 12 before running the baseline verification.

Next: have him identify init(), start(), loop(), and stop(), and explain why follower.update() is in loop(). Finally, ask him to sketch two moves and explain what condition should start the second move. Help him implement one small step at a time.

## One source, two builds

| Location | Purpose |
|---|---|
| shared/src | The actual lesson OpModes and path logic, compiled unchanged into both targets |
| sim-profile/src | LearningRobot factory selects simulator hardware/configuration |
| robot-profile/src | Same factory API selects the copied real robot configuration |
| simulator | Pinned upstream simulator and mock FTC hardware |
| robot-build | Independent snapshot of the current robot project; includes shared sources through TeamCode/build.gradle |

**Verify-Lab.cmd** runs actual simulator physics tests for left, forward, turn, Pinpoint heading reset, and the upstream Ivy example including STOP. It expects the original 12-inch lesson setting.

**Build-Robot.cmd** builds the FTC Android APK from the independent robot-build copy. It does not install anything on the robot. Output: robot-build/TeamCode/build/outputs/apk/debug/.

Open this folder as a Gradle project for desktop editing. If Android Studio treats its Java application unusually, the scripts remain the supported launch/build route. Open robot-build in a separate Android Studio window to inspect the FTC app; shared files live outside TeamCode but are included by its source configuration.

## Verified and limitations

Verified locally: simulator execution, all baseline checks, and FTC SDK 12.0.0 APK compilation. Both targets use Pedro core/revhub 3.0.1 and Ivy core/pedro 1.1.1. This proves these lesson source files transfer and compile; it does not prove a physical robot follows the path accurately.

The simulator uses mecanum dynamics and a simulated Pinpoint API. It does not reproduce real Pinpoint sensor fusion/calibration, wiring, pod slippage, battery behavior, vision hardware, or your flywheel. Robot-specific names, directions, offsets, and tuning belong in the robot profile/configuration, not the shared path logic. Never replace real robot tuning with simulator tuning.

A specific issue found in the copied robot Constants.java: offsetUnits is INCH while offsets are 158 and -84. These are implausibly large for this robot and may have been measured in millimeters. Verify the measurements and units before physical testing. They were left unchanged, as was the original project. Several algorithm values are also explicitly marked as example/untuned values in that file.

See MIGRATION.md for moving existing robot code into this workflow. See PROVENANCE.md for upstream/version details.
