# Panels on Mimir

FullPanels 1.0.19 is included in the Robot Controller APK. Build and Run `TeamCode`
onto the Control Hub, join its Wi-Fi, and open http://192.168.43.1:8001 using HTTP.
Refresh the browser after deploying the new APK.

## Arrange the dashboard

Use Panels' layout editor to add the widgets you want. The layout belongs to the
browser; deploying robot code does not arrange it automatically.

| Widget | What is available |
| --- | --- |
| Telemetry | Existing Driver Station messages, robot pose, both drivers' raw joystick axes, triggers, and button states |
| Graph | Numeric telemetry, including wheel speeds, shooter target/measured/error, loop frequency, heading, joystick axes, and vision offsets |
| Field | Robot outline and heading, recent cyan trail, gray autonomous routine preview, yellow active path |
| Limelight feed / dashboard | Built-in Limelight USB network proxy, video, and camera configuration |
| Lights | Shooter ready and Limelight target indicators |
| Configurables | TeleOp shooter target, flywheel tuning P/F gains, field trail/radius, and TeleOp starting coordinates |
| Capture | Record/replay the dashboard data sent by the robot |
| Battery / Pinger | Built-in battery and connection monitoring |
| OpMode Control | Built-in INIT, START, and STOP controls |
| Gamepads | Built-in browser-connected gamepad tools; Driver Station input monitoring is also provided through Telemetry and Graph |
| Integrated Docs | Packaged Panels documentation, available without Internet |

## Suggested graphs

- Shooter: `Shooter requested (ticks/s)`, `Shooter measured (ticks/s)`,
  `Shooter tracking error (ticks/s)`.
- Driving: `Driver 1 right stick X`, `Driver 1 right stick Y`,
  `Driver 1 left stick X`, and the wheel speed values.
- Localization: `Pose X (inches)`, `Pose Y (inches)`, `Pose heading (degrees)`.
- Vision: `Limelight TX (degrees)`, `Limelight TY (degrees)`,
  `Limelight target area (%)`, and `Limelight frame age (ms)`.

Values normally refresh around 10 times per second. Buttons are 0 or 1, making them
graphable. Brief presses between samples may not appear. Inputs are the actual
Driver Station values; the dashboard does not replace them with browser inputs.
Processed drive commands after the deadzone and field rotation are also reported
by the main TeleOp.

## Field coordinates

The Field output uses the Panels Pedro Pathing preset and inches, matching the
autonomous path definitions. Autonomous routines with a Pedro follower preview
their full route during INIT and draw their measured position while running.
Select the correct season's field image in the widget.

TeleOp uses its existing Pinpoint localizer even when position hold is off.
Its starting pose defaults to `(0, 0, 0 degrees)` relative to INIT. For an absolute
field position, set `RobotPanels.TELEOP_START_X`, `TELEOP_START_Y`, and
`TELEOP_START_HEADING_DEGREES` in Configurables **before initializing TeleOp**.
Changing these after INIT requires stopping and initializing again. Resetting the
driver's IMU yaw does not relocate the Pinpoint pose.

Pedro localization tests use their own test origin. Timed `AutoLeft`/`AutoRight`,
the variable example, flywheel tuning, and motor-direction tests have no pose
source, so they do not draw a robot position. A dashboard never invents a pose
for those modes.

## Limelight

The configured hardware name is `limelight`; the robot selects AprilTag pipeline
8. The existing FullPanels Limelight proxy connects to `172.29.0.1` over the
camera's USB connection. Keep the Limelight plugged into the Control Hub.
The native Limelight feed/dashboard widget is the camera view to use; the general
webcam Camera Stream widget needs a separate webcam/VisionPortal source, which
this robot does not currently have.

Main TeleOp starts Limelight polling at START. The shooting autonomous modes also
poll during INIT. Diagnostics distinguish disconnected, waiting, stale, wrong
pipeline, no target, and detected AprilTag states. TX/TY/area are only published
for fresh valid tags in pipeline 8, rather than plotting missing tags as zero.
The empirical range estimate retains the robot's existing calibration and is
not a general 3D distance measurement. Using the camera dashboard to change away
from pipeline 8 makes robot vision report `Wrong pipeline`.

## Live settings

`Mimir_2026_Mec.SHOOTER_TARGET_VELOCITY` is in encoder ticks/second.
`flyWheelTuning.P` and `.F` update the flywheel tuning mode's existing PIDF loop.
Changes in Configurables are runtime settings; copy values you want to keep back
into source code. Dashboard monitoring does not schedule extra robot movement.

## Verification on the robot

After installing, INIT the main TeleOp and confirm the Field marker appears.
START, move a joystick, and check the raw/processed input values. Move the robot
and confirm the trail and heading match physical motion. Check Limelight video,
then a tag in pipeline 8, and confirm the ID, fresh-target indicator, and offsets.
Check shooter target/measured graphs and the ready light. STOP should stop the
mechanisms and clear the dashboard status lights. Hardware and camera behavior
must be checked on the robot; an APK build cannot validate them.
