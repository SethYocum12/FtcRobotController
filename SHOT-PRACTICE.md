# Practice instructions — do one step at a time

## AprilTag auto-rotate and manual feeding

This follows the behavior in [Brogan's mecanum AprilTag auto-rotate video](https://www.youtube.com/watch?v=dATlviyccY0): click the **right stick** to
toggle rotation toward the visible alliance goal tag while your drive stick
still controls translation and strafing. D-pad Up is a second toggle. Click
either again to turn auto-rotate off. When no alliance tag is visible, the
robot lets the left stick turn manually. The Limelight is rear-facing, so the
code flips camera TX to match the robot's turn direction.

It uses the BIOBUZZ alliance cell tags: red IDs 30–37 and blue IDs 38–45.
When multiple same-alliance tags are visible, it keeps using the same tag while
that tag remains visible. **Y** toggles the shooter, **B** manually toggles the
indexer, and holding **X** reverses it. B does not wait for tag visibility,
alignment, or flywheel recovery. Start with the wheels clear and B off; for
shooting, begin with one ball and be ready to stop or reverse the indexer
yourself.


## Read this first if the robot turns or you cannot find the saved number

`N` just means a NUMBER. You will see `Saved sample 1`, then `Saved sample 2`.
You will not see a letter N. Look for `Sample status` and `Saved samples` at the TOP of the practice DS screen.

For now, test MANUAL shooting so we can check whether auto aim is causing the turn:

1. Press STOP.
2. Select `Mimir_2026_Mec`, press INIT, select your alliance with Left/Right.
3. Press Down once and confirm `Shot practice` says ON.
4. Press START. Do NOT press D-pad Up or the left bumper. Auto aim and hold
   should both be OFF. Check the DS display if you can see those lines.
5. Aim the robot yourself and release the drive sticks.
6. Press Y to turn the shooter on. Wait for the speed to settle.
7. Press B to feed. Watch whether the WHOLE ROBOT turns.
8. Press B again to cancel feeding. Keep Y on for the snapshot.
9. With an alliance tag still visible, press Down once and release it.
10. Find `Sample status`. It should say `Saved sample 1` (or another number).
    Missing range/tag is now saved as a diagnostic row. If it says `Save failed`, photograph the exact message.
11. Write down whether it still turned with auto aim OFF. Then press STOP.

Auto aim can command turns while shooting. If it turns with auto aim OFF too,
we need another check; we have not established the cause from this observation.

Open this on the Mac before connecting to the robot Wi-Fi.
Ask someone to film while you drive. Bring a tape measure and something to write on.
You do not need to do the math or count video frames.

## Part 1 — Start practice mode

1. Connect the Mac to the robot's Wi-Fi.
2. Upload the new code using Android Studio's Run button.
3. On the Driver Station, select `Mimir_2026_Mec`.
4. Press INIT.
5. Select the alliance: D-pad Left for RED, or D-pad Right for BLUE.
6. Read `Alliance` on the Driver Station. Make sure it is correct.
7. Press and release D-pad Down ONCE.
8. Read `Shot practice`. It must say ON. If it says OFF, press Down once more.
9. Press START.

D-pad Down now SAVES readings. It no longer switches practice mode.
If you STOP and start a new TeleOp, repeat Part 1 and note that it is a new session.

## Part 2 — First check: can it see and aim at a tag?

1. Leave the shooter OFF. Do not press Y yet.
2. Put the robot near a shooting spot, with its REAR shooter toward the basket.
3. Look at the Driver Station for `Selected aim tag`.
4. Write down that number. Use this same tag for the shooting tests below.
5. If there is no selected tag, fix tag detection before continuing.
6. Press and release D-pad Up once to turn auto aim ON.
7. Watch the robot turn toward the target.
8. If it turns the wrong way or shakes, turn auto aim OFF with D-pad Up.
   Take a video and tell me. Use manual aiming for the stationary shot tests.
9. If it aims smoothly, leave auto aim ON for now.

The robot can still move with your drive stick while auto aim is ON.
Do not press B for the movement checks later.

## Part 3 — Mark one sideways shooting line

1. Choose one shooting distance where your existing shooter speed works well.
2. Keep the SAME shooter speed for all the tests below.
3. Mark a short line across the floor, left/right in front of the basket.
4. Mark a CENTER spot, a LEFT spot and a RIGHT spot on that line.
5. Start with a small sideways span. We can expand it after testing.
6. Put the robot at CENTER with its rear shooter toward the basket.
7. Keep auto aim OFF for shooting measurements until we fix the turning problem.
8. Record the selected tag ID. Use that same tag for the tests when possible.

## Part 4 — Shoot while STOPPED at CENTER, LEFT and RIGHT

1. At CENTER, aim by hand and release the sticks.
2. Measure camera-lens to tag-center distance. Write down the inches and tag ID.
3. Turn Y shooter ON. Keep B feeding OFF until speed settles.
4. Photograph the target speed, actual speed and aim readings on DS.
5. Press Down and note the saved sample number. If range is NaN, the new code
   still saves it and labels the missing range. Write "range missing" in your notes.
   A missing range still BLOCKS moving-shot feeding.
6. Start a side-view video showing the shooter exit and basket opening together.
   Say "CENTER, speed ___" aloud. Keep the original recording.
7. Press B to feed. Film several shots and count balls IN / balls LAUNCHED.
8. Press B again to cancel feeding. Stop the video. Write down misses or jams.
9. Turn Y OFF before moving.
10. Move to LEFT on the SAME line. Aim by hand and repeat steps 1–9.
    Keep the SAME shooter speed. Call its video "LEFT".
11. Move to RIGHT and repeat. Call its video "RIGHT".
12. If the same speed does not work at the ends, shorten the line and test again.

These stationary videos also give us flight-time measurements. You do not need
to count video frames yourself. Do not change to close/far distance tests now.

## Part 5 — Test auto aim while moving, WITHOUT feeding

1. Cancel B and leave Y shooter OFF.
2. Put the robot back at CENTER. Confirm it sees an alliance tag.
3. Film the robot and basket, and photograph the DS aiming readings.
4. Press D-pad Up to enable auto aim.
5. Move slowly toward LEFT along the marked line, then toward RIGHT.
6. Let auto aim control the turn; you control the drive stick.
7. If it turns the wrong way, shakes, or tracks the wrong place, turn auto aim
   OFF. Save that video and tell me what happened. Do not feed through this issue.
8. If it tracks smoothly, save that video too. We still need to calibrate and validate motion compensation
   before trying to feed while moving.

## Part 6 — Film the feeding delay for shoot-while-moving prep

1. Return to a stationary shooting setup that worked.
2. Keep B OFF. Turn Y ON and let the shooter reach speed.
3. Ask the filmer to take a CLOSE-UP showing the indexer and where a ball leaves
   the shooter. The basket does not need to be in this close-up.
4. Start recording. Call this video "INDEXER CLOSE-UP".
5. Press B to request feeding and film the indexer actually moving a ball through.
6. Cancel B after the shot group.
7. Stop recording and turn Y OFF.

I will help inspect the time from actual feeding to release. The B press alone
is not that time, because the code might wait before feeding starts.
The CENTER/LEFT/RIGHT side videos are what we use to inspect ball flight time.

## Part 7 — Measure LEFT and RIGHT movement, WITHOUT shooting

1. Cancel B and turn Y shooter OFF.
2. Use the same sideways line. Measure the distance between two marks on it.
3. Leave room before and after the marks to accelerate and stop.
4. Photograph which way the robot's front faces. LEFT/RIGHT mean the robot's
   left/right in that photo, not the filmer's.
5. Ask someone to film from a fixed position showing the robot and both marks.
6. Start moving slowly before the first mark. Move steadily through both marks.
7. Press Down during steady movement and note the sample number. It now saves
   velocity readings even without a tag/range; missing vision is explicitly marked.
   Photograph the displayed velocity and heading if file storage fails.
8. Stop past the second mark and stop the video.
9. Write the direction, measured travel distance and video name down.
10. Repeat in the opposite direction.
11. Keep B canceled throughout. Do not shoot while moving yet.

If auto aim rotates the robot during the run, note that in the video. We need
its heading along with the velocity readings to relate movement to the basket.
Forward/backward shooting and automatic RPM changes are outside this first test.

## Part 8 — Take pictures and measurements

1. Turn the shooter OFF and cancel feeding.
2. Photograph the BACK of the robot, showing camera and shooter.
3. Photograph the SIDE of the robot, showing camera and shooter.
4. Find the robot's center. From a top view, that is halfway across its length
   and halfway across its width. If the outside dimensions are 18 by 18 inches,
   the center is 9 inches from each side.
5. Measure where the camera LENS is compared with that center: how far forward
   or backward, and how far left or right. Write the directions and inches down.
6. Measure the lens's height above the floor.
7. Do the same three measurements for the point where a ball LEAVES the shooter.
8. Take a picture showing how the camera is tilted or turned.
9. Photograph the selected field tag and basket opening together.
10. If practical, measure how far the opening is above/below, left/right and in
    front of/behind the tag center. Label the directions and picture viewpoint.

A photo of the tape and handwritten numbers is fine. Say "approximate" if a
measurement is approximate. Say "not measured" if you could not measure it.

## Part 9 — Get the saved files onto the Mac

The CSV files are just spreadsheets of the robot's numbers. You do not need to
open or understand them. One press of D-pad Down adds one row to the current file.
They do not tell us whether a shot scored; that is why we also need your notes.

1. Connect the Mac to the robot's Wi-Fi.
2. Open the terminal inside Android Studio.
3. Run this line and press Enter:

```sh
~/Library/Android/sdk/platform-tools/adb connect 192.168.43.1:5555
```

4. Run this line and press Enter:

```sh
~/Library/Android/sdk/platform-tools/adb -s 192.168.43.1:5555 pull /sdcard/FIRST/shot-practice ./shot-practice-results
```

5. Find `shot-practice-results` in your project folder.
6. Keep the `.csv` files in it. Send all this practice's files if you restarted TeleOp.
7. If this fails, keep the notes and DS photos. Tell me what the terminal said.

## Part 10 — Send these to me

1. Original CENTER, LEFT and RIGHT stationary-shot videos at the same speed.
2. The INDEXER CLOSE-UP video.
3. Movement videos and the photos showing the robot's starting orientation.
4. The CSV files, or DS photos if saving failed.
5. Photos and measurements from Part 8.
6. Your notes connecting the sample numbers to the videos and scores.
7. The recording frame rate if known. Say "unknown" if you do not know.

Keep original phone videos. Do not change their speed or export edited versions.
I will help check frames and calculations. Hidden release/arrival moments or
unclear video timing may mean we need another recording.

Use this note for EACH shooting trial:

```text
CENTER / LEFT / RIGHT:
Video name:
Saved sample number and session:
Selected tag ID:
Tape camera-to-tag distance, inches:
Camera-tag range shown on DS:
Shooter speed setting:
Balls IN / balls LAUNCHED:
Miss direction or jams:
Auto aim ON or OFF:
```

Use this note for EACH movement video:

```text
Video name:
LEFT / RIGHT:
Measured travel distance, inches:
Saved sample number and session, or "did not save":
Starting orientation: see photo ___
Any problems:
```

If you restart TeleOp, sample numbers start over. Mark which session a note belongs to.

## If time runs out — stop here and save what you have

You do NOT have to finish every test. Useful partial results are enough to review.
When there is about ONE MINUTE left, stop starting new tests and do this:

### 1. Stop the robot

1. Release the drive sticks and X reverse button.
2. Press STOP on the Driver Station. This turns off the shooter, intake, indexer
   and position hold. Do not rely on remembering whether B or Y was toggled on.
3. Wait for the wheels and shooter to finish stopping before touching the robot.
4. Leave the Control Hub powered on for now so you can copy its files.

If anything is behaving unexpectedly, press STOP immediately. Saving another
sample can wait; do not keep a test running just to finish the notes.

### 2. Keep the readings you already saved

1. Every time DS said `Saved sample N`, that row was written and its file closed.
   Pressing STOP does not delete those saved rows. There is no extra Save button.
2. Do NOT press D-pad Down after STOP expecting another snapshot. It only saves
   during the running OpMode with practice ON.
3. Write down your last saved sample number and which session it was from, if known.
4. If you forgot to save a trial, mark it "no snapshot" and keep its video/notes.
   Do not guess missing numbers or restart the shooter just to recreate them.

### 3. Save the phone videos and notes

1. Stop recording on the phone.
2. Open Photos and check that the latest video appears and plays.
3. Keep the ORIGINAL videos on the phone, including the slow-motion originals.
4. Photograph your handwritten notes so there is another copy.
5. Write down which tests you finished and which you skipped.
6. Match each video name to its saved sample number if you can. If you do not
   know which sample matches, write "not sure" rather than guessing.

### 4. Copy the robot files while it is still powered on

1. Keep the Mac connected to the robot Wi-Fi. The OpMode can stay STOPPED.
2. Follow Part 9's two terminal commands to copy the CSV files.
3. Check the terminal for a successful copy, not an error.
4. In Finder, open the project folder and then `shot-practice-results`.
5. Check that there are `.csv` files inside. Open one in Numbers or a text editor
   and check that it has rows of readings below the column names.
6. Copy the ENTIRE `shot-practice-results` folder into a new folder on your Mac's
   Desktop. Name the new folder `Shot Practice` plus the practice date.
7. Copy your phone's ORIGINAL video files into that same dated folder when you
   can. Keep the phone copies too. Open a copied video on the Mac to check it plays.
8. Put the notes photos and mounting pictures in the same folder.
9. Once the CSV copy is checked and the robot is stopped, turn the robot power OFF.

Copying the CSV files does not remove them from the Control Hub. Keep both copies.
You do not need internet to save files on the Mac or phone.

### 5. If you must leave before copying

1. Press STOP and let the robot's moving parts stop.
2. Stop phone recording and keep your videos and notes.
3. Turn robot power off normally. Previously saved CSV rows stay on the hub;
   they are not only stored in TeleOp's memory.
4. Write yourself a reminder: "CSV files still on Control Hub — copy them later."
5. Next time, power the hub on, connect the Mac to its Wi-Fi, and follow Part 9.
   You do not need to run the shooter or repeat the tests to copy the old files.
6. Send the videos and notes first if needed. Tell me the CSV files will come later.

### 6. What to send if you only finished part of practice

Send the videos, CSV files or DS photos, and notes that you DID collect.
Add one sentence such as:

```text
We finished CENTER and LEFT stationary shots. We did not get RIGHT shots or movement videos.
The CSV files are copied / still on the robot.
```

Prioritize useful stationary shot results first. Moving shots still need
integration and robot testing after we check the measurements.

## What has to be entered before sideways shooting can be enabled

This is a code-setup checklist for after we review your measurements. You do not
need to calculate these during practice. Settings live in
`mechanisms/SidewaysShotPlanner.java`, in `Calibration` / `CALIBRATION`.

- The ONE tested flywheel speed, selected tag ID, ball flight time and feed-to-release delay.
- Basket aim-point X/Y, the sideways line's position/direction, and its usable
  left/right limits in the SAME Pedro coordinate frame as the robot pose.
- Shooter exit forward/left offsets from robot center.
- Successful stationary raw TX, and the tested sign relating TX to a CCW turn.
- Verification that the camera gives valid range, and that the aim correction
  works from CENTER, LEFT and RIGHT within this small corridor.

The field coordinates must be referenced to the exact physical START pose used
for TeleOp localization. Starting elsewhere without correcting that pose makes
this calibration invalid. Camera/shooter parallax and the tag-to-basket offset
must be checked across the corridor; the small constant-angle correction is an
approximation that must pass these tests.

Initial test limits are 6 in/s sideways, 1 in/s normal to the line, 1 inch off
line, 5 degrees/s turning, 6 in/s² acceleration, and 0.2 seconds steady movement.
Frames must be at most 100 ms old. These are conservative starting limits, not
measured performance claims. Change them only after testing.

After the settings are populated, `geometryAndTimingVerified` and `enabled`
remain false until their checks pass. There are no invented flight times or
geometry values in the shipped profile. The current right-stick auto-rotate
tracks the selected alliance AprilTag while driving. B manually toggles the
indexer in TeleOp regardless of range, aim readiness, or flywheel recovery.
X reverses.
Autonomous shooting remains separately controlled by its scripted tag and speed
rules because there is no driver B input during auto.

## If the range still says NaN after uploading the latest code

The code now checks TWO 3D pose fields for the SAME selected tag. Either can
supply the camera-to-tag distance. It will not use an estimated number just to
hide missing camera data. Supported fields are documented here:
https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification

1. Upload the latest code with Android Studio when connected to the robot.
2. Enter practice mode, select the correct alliance and press START.
3. Leave the shooter and auto aim OFF while checking the readings.
4. Point the camera at an alliance tag.
5. Read `Range status` near the top of the practice screen.

- `Distance OK`: check the number against the tape before trusting its scale.
- `No fresh alliance tag`: check selected alliance, tag visibility and camera status.
- `Tag seen, no 3D distance`: neither camera/tag pose contains a usable distance.
  Press STOP. Check pipeline 8's Full 3D targeting and the real tag size in the
  Limelight interface when you can access it. Tell me the exact message or send
  a picture. This message alone does not prove which setting is wrong.

Even with missing range, Down saves a diagnostic snapshot for later review.
Missing range does not prevent manual TeleOp feeding with B.

## If the first shot is good but later shots go low

The manual TeleOp indexer no longer checks flywheel recovery before feeding.
Autonomous scripts still check shooter speed and pause their scripted feed.

1. Wait for actual flywheel speed to reach 97–103% of target and stay there for
   0.25 seconds. Then the indexer is allowed to feed if the other aim checks pass.
2. If the speed falls below 97% or rises above 103%, feeding pauses immediately.
3. After the speed recovers, it must settle in that band for another 0.25 seconds.
4. B stays requested during a recovery pause. Press B again to cancel it. Turning
   Y off also cancels the request. X reverse remains manual.
5. Look at `Flywheel feed check`. Waiting/recovery messages explain a pause.
   `Flywheel speed drops` counts speed dips after readiness, NOT balls fired.
6. Film a stationary group of several shots. Record target and actual velocity,
   which shots went low, and whether the indexer paused. Do not use moving-shot
   testing to diagnose the stationary shooter first.

The auto still leaves at its 20-second cutoff. Its tag trigger stays latched,
periodic reverse schedule is preserved, and speed recovery can pause indexing.
This can reduce feeding while the flywheel is slow. It cannot stop a ball already
engaged in the shooter, fix a mechanical jam, or prove that the chosen speed and
launch angle are correct. If low shots continue while speed readings look good,
send the video and readings so we can check those causes next.

Checks: `python3 tools/check_flywheel_feed.py`.

## Camera fix confirmed on November 6

Pipeline 8 had Full 3D tracking OFF (`fiducial_skip3d=1`). It is now ON
(`fiducial_skip3d=0`), saved on the camera, with pipeline 8 selected.
Live readings returned valid 3D positions for tags 42, 44 and 45.
For example, tag 42 reported 60.16 inches at the time of the check.
That initial repair verified that numbers were being produced and left tag size
at 101.6 mm. Later, your home tag's black square measured 3.25 inches, so pipeline
8's marker size was changed to **82.55 mm** and verified from disk. After that,
you reported **52 inches** on the DS at CENTER, close to the **53.4-inch** estimate
from your tape measurements. This passes the rough home distance check. Match
marker size to the physical tags you use: 82.55 mm is for these 3.25-inch home
prints; a 4-inch black square needs 101.6 mm.
Backups and verification readings are in `limelight-backups/2026-11-06/`, with
the later size correction in its `marker-size-correction/` folder.

Follow-up IMG_0784.MOV gives a **provisional 0.46-second airtime estimate** at
your confirmed 1300 ticks/s target. Its saved 30 fps frames are interpreted as
8-times slow using your 240 fps capture report. The oblique view leaves crossing
uncertainty; this estimate does not by itself enable moving-shot feeding. See
`practice-analysis/2026-11-06/IMG_0784-flight-analysis.json` and `RESULTS.md`.

Reconnect the camera to the Control Hub to test through the Driver Station.
The camera setting is already saved; no new robot upload is needed just for it.

The robot code also has faster auto-aim tracking with the B feed request OFF:
1.5 times the old turn command, capped at 0.22. With B requested it keeps the
normal slower turn response. Upload the latest robot code to get this change.
Test at a small aiming error with feeding OFF first. If it oscillates, stop and
report it so we can reduce the tracking gain.
