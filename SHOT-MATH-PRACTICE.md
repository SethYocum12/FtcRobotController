# Shot math practice — four-shot first pass

**Keep this file open on the Mac before switching to the robot Wi-Fi.**
Bring the Mac, Driver Station/controller, yellow balls, tape/tape measure,
and a phone plus a helper to film and count.

**First pass: 4 individual ball launches at each of 5 positions = 20 shots.**
Use one fixed shooter setting and shoot while stopped. The three centerline
positions provide an initial flight fit. LEFT/RIGHT results stay separate for
checking the prediction. Four shots give a rough first check; larger sets help
show inconsistency.

**If time allows, extend LEFT and RIGHT to 10 shots each.** Keep all the initial
four shots in each count and add six more, using the same setting and angle.
With four at FAR/MIDDLE/NEAR and ten at LEFT/RIGHT, that is **32 shots total**.
The first 20 shots alone do not complete validation of the broader prediction.

The robot records the numbers. A helper counts shots and films. You drive,
aim, and feed manually. Rumble checking is automatic.

## Before shooting

1. Plug the Limelight into the Control Hub using its normal robot USB connection.
   Connect the Mac to the robot as you normally do for Android Studio uploads.
   Upload the updated project with **Run**. On the Driver Station select
   **Mimir_2026_Mec**, then **INIT**.
2. Check your alliance: D-pad Left = red; Right = blue.
3. During INIT, press and release **D-pad Down once**. Find the exact line
   **Shooter speed practice: ON** on the Driver Station.
4. Press **START**. Check that **Recording** appears on the Driver Station.
   With the hive visible, also check **3D DATA: READY**. If it stays at WAIT,
   save a Driver Station photo and fix that before collecting the shot set.
5. Use only the yellow POLLEN balls for this set. Keep the same shooter target
   for every position. The current code starts with your normal TeleOp target;
   the log records its exact value. If you adjust it to find a working shot,
   do that before starting position 1, then leave it alone for all five.
6. Find a manually aimed spot that scores at the real hive. Mark its robot-center
   location with tape. Call it **FAR**. This can be your usual spot near the wall.

“Prediction needs practice recordings” is expected. The flight model is not
trained yet. **Cue source** shows whether feedback comes from an existing saved
spot or from a validated trajectory model.

## The five positions

Use floor measurements from the robot-center tape marks. This plan does not
require driving farther back toward the wall than your normal shooting spot.

| Position number | Name | Put the robot here | First-pass launches |
| --- | --- | --- | ---: |
| 1 | FAR | Your normal working shooting spot | 4 |
| 2 | MIDDLE | 6 inches closer to the hive than FAR | 4 |
| 3 | NEAR | 12 inches closer to the hive than FAR | 4 |
| 4 | LEFT | 12 inches left of MIDDLE, looking toward the hive | 4; extend to 10 if time allows |
| 5 | RIGHT | 12 inches right of MIDDLE, looking toward the hive | 4; extend to 10 if time allows |

At LEFT/RIGHT, manually turn the rear shooter toward the opening. Keep the
shooter setting fixed. Scores and misses both help; do not move a marked
position or change power just to improve its count.

## Do this at EACH position

1. Move to the tape mark and aim by hand. Release the drive/turn sticks.
2. Press and release **D-pad Down once**. The Driver Station should show the
   next **PRACTICE POSITION** number. Say that number and position name on video.
   Check **3D DATA: READY** at this position too.
3. Film the shooter exit, ball flight, and whole hive opening together. A clear
   side/quarter view is useful. Keep the camera still; use 60 fps or faster if
   available. Keep the original video.
4. Press **Triangle** to run the shooter. Wait for **FLYWHEEL: Flywheel ready**.
   Use a few practice balls to find a good turning angle if needed. Say
   **“practice shots”** on video, then **“starting the counted set”** when ready.
   Keep that angle for the counted launches. Do not
   press Down again for the same position.
5. Feed **one ball at a time** with Circle, then stop feeding with Circle again.
   Loading one ball at a time makes this test easier. Before each next launch,
   wait for **Flywheel ready** again.
6. Count **4 actual launches** for the first pass. Note how many went IN and which misses were
   low, high, left, right, short, or over the back. Count jams separately from
   launches. Keep video of misses as well as makes. Record the complete set,
   even if it is 0/4 or 3/4; do not restart the count to obtain a perfect result.
   At LEFT/RIGHT, you can add six more shots to reach ten at each position.
7. If the hive tips, pause feeding and have it reset to the same upward-facing
   cell before continuing. Include the tip/reset in the notes/video.
8. Stop feeding and turn the shooter off before moving to the next mark.
   Recording continues automatically between positions.

You do not need to press Up to train the math. **Up** still confirms a working
spot for the temporary saved-spot cue, which is a separate source of feedback.

## Write only this much

| Position | IN / total launches | Misses / jams | Original video filename |
| --- | --- | --- | --- |
| 1 FAR | | | |
| 2 MIDDLE | | | |
| 3 NEAR | | | |
| 4 LEFT | | | |
| 5 RIGHT | | | |

Example: `1 FAR: 3/4. Shot 4 low. One jam before a launch.`
For extended validation, an example is `4 LEFT: 9/10, including the first four. Shot 6 right.`
Camera measurements, per-tag 3D positions, target/actual flywheel speed,
odometry, motion, and feed/reverse commands are recorded in CSV automatically.
The recording counts frames, **not balls**; your count and video provide shot outcomes.

## Finish, or stop early if time runs out

1. Stop feeding with Circle. Turn the shooter off with Triangle.
2. Press **STOP** on the Driver Station. This flushes and closes the CSV file.
3. Keep the original videos and your results table.
4. Copy the log folder from the Control Hub. If ADB is connected as it is for
   Android Studio uploads, run these from the project Terminal on the Mac:

```sh
mkdir -p shot-practice-results/real-hive
~/Library/Android/sdk/platform-tools/adb pull /sdcard/FIRST/settings/mimir-shot-math shot-practice-results/real-hive/
```

Send the copied folder, original videos, and your five counts. The CSV file
path is also shown on the Driver Station. Logs are flushed about once a second,
after position markers, and on STOP. Each session has its own filename.

If time is short, prioritize **FAR, MIDDLE, NEAR** and send what you finish.
Those can fit the launch curve; LEFT/RIGHT validation will still be needed
before the broader predicted-shot cue is enabled. Completed data need not be repeated.
Aim for ten validation launches at LEFT and ten at RIGHT before calling that
check complete. Extra validation can be collected later at the same unchanged setup.

## What the math uses

The trajectory is modeled as:

```text
position(t) = shooter_exit + launch_velocity * t + gravity * t^2 / 2
```

The code solves where this curve enters the tilted opening plane. It checks
the actual pentagonal face, with clearance for the ball and fitted error.
It can evaluate unsaved positions within the calibrated distance span.

Fitting uses measured ball crossing heights from the videos and logged
geometry. A successful shot is not automatically assigned the opening's
center height. Misses and mechanical problems remain visible in the dataset.

Known camera/muzzle position measurements are retained. Camera rotation,
tag-to-opening mapping, launch parameters, and LEFT/RIGHT validation must be
verified from the field data before the math can control rumble. Until then,
the file `FIRST/settings/mimir-stationary-shot-math.properties` contains an
unverified template. Confirming a saved spot does not activate that model.

The offline fitter is `tools/fit_stationary_shot_math.py`. It uses the production
Java math, excludes validation rows, and produces an unverified candidate for
review. You do not need to prepare that fitter input yourself.

Reference frames follow the [Limelight coordinate documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems).
The known opening outline is from [FIRST Arena TU06](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09),
Figure 9-11. The local FTC SDK 12.0 BIOBUZZ tag library provides the four tag
member offsets; their mapping into this sideways camera's frame remains a
field-verification step.
