# Practice instructions — do one step at a time

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

## Part 3 — Test the CLOSE shooting spot

1. Choose a close spot your robot could actually shoot from.
2. Stop the robot and keep its rear shooter toward the basket.
3. Check `Selected aim tag`. Write the tag number down.
4. Measure straight from the CAMERA LENS to the CENTER OF THAT TAG.
   Write the inches down. Do not measure along the floor instead.
5. Read `Camera-tag range (in)` on the Driver Station. Write that number down too.
6. Press and release Y once to turn the shooter ON.
7. Read `Shooter target` and `Shooter velocity`. Wait for the actual speed to settle.
8. B should still be OFF. Press and release D-pad Down ONCE.
9. Look for `Saved sample N`. Write down N. This saved the robot's numbers.
10. Tell the person filming: "This is CLOSE, sample N."
11. Have them stand to the SIDE where they can see BOTH the shooter exit and the
    basket opening. Keep the phone still and show the whole flight of the ball.
12. Start recording. Slow motion is helpful if available. Keep the original file.
13. Press and release B once to request feeding.
14. With auto aim ON, wait for alignment, 97% flywheel speed and two seconds still.
    Feeding should start once those checks pass.
15. Film several shots. Count how many were launched and how many went in.
16. Press and release B again to cancel feeding. Check the indexer stops.
17. Stop the video.
18. Write down the video name, the score, and whether misses were short, long,
    left, right, or caused by a jam.

If the snapshot does not save, check the alliance/tag/range. If it still will not
save, photograph the Driver Station numbers and write "snapshot failed".

## Part 4 — Change the speed and try again

Do this if the CLOSE shots need a different speed.

1. Make sure B feeding is OFF. Leave Y shooter ON while adjusting.
2. D-pad Right raises the speed by 25 ticks/second per press.
3. D-pad Left lowers the speed by 25 ticks/second per press.
4. Read the new target on the Driver Station. Wait for the actual speed to settle.
5. Press D-pad Down once. Write down the NEW saved sample number.
6. Start a NEW video and say its sample number aloud.
7. Press B to request feeding. Film and count the shots.
8. Press B again to cancel feeding. Stop the video.
9. Write down the score and misses for this setting.
10. Repeat until you find a useful setting, or move on if time is running short.

If the tag-centered aim misses sideways, cancel B and turn auto aim OFF with
D-pad Up. Aim by hand, then repeat the trial. With auto aim OFF, B feeds manually
while Y is on; it does not wait for the auto-aim checks. Label the trial
"manual aim". Save its readings while the tag is still visible.

If a ball jams, cancel B. Hold X to reverse the indexer, then release X.
Write "jam" in the notes so we do not mistake it for a bad shooter speed.

## Part 5 — Do MIDDLE and FAR spots

1. Cancel B feeding.
2. Turn the shooter OFF with Y before moving to the next spot.
3. Move to a useful MIDDLE shooting spot.
4. Check you are seeing the SAME selected tag as the close test.
5. Repeat Part 3 and Part 4 there. Call its videos "MIDDLE".
6. Cancel B and turn Y off again.
7. Move to a useful FAR shooting spot.
8. Check the same tag again.
9. Repeat Part 3 and Part 4. Call its videos "FAR".

Keep separate notes for each sample. If the selected tag changes, write the new
ID down; do not mix that trial into the old tag's measurements.

Do NOT turn Y off before saving a shooting setting. Its saved target would be zero.

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
The CLOSE/MIDDLE/FAR side videos are what we use to inspect ball flight time.

## Part 7 — Check movement WITHOUT shooting

The moving-shot corrections are not connected yet. Do not shoot while moving.

1. Make sure B is canceled and Y shooter is OFF.
2. Choose a straight line on the floor where the robot can still see an alliance tag.
3. Measure a 24-inch section of that line. Mark its start and end with tape.
4. Leave room before the first mark and after the second mark.
5. Take a photo showing the robot's FRONT and which way it faces along the line.
6. Ask the filmer to stand still and show BOTH tape marks and the robot.
7. Start recording. Say "FORWARD movement, 24 inches".
8. Start driving BEFORE the first mark so the robot is moving steadily when it
   enters the measured section. Keep its heading steady.
9. Press D-pad Down once while it is moving steadily. Do not stop to press it.
10. Drive past the second mark, then stop.
11. Check whether a sample saved. Write the number down, or write "did not save".
12. Stop recording. Note the video name and the 24-inch measured distance.
13. Repeat steps 5–12 moving BACKWARD along the line. Say "BACKWARD" in that video.
14. If time allows, repeat with the robot moving LEFT, then RIGHT without turning.
    Directions mean the robot's left/right, not the filmer's.
15. If there is still time, turn the robot about 90 degrees, photograph its new
    orientation, and repeat one direction. This helps check the coordinate axes.

You may need to choose another line or direction to keep a tag visible. If it
cannot see a tag, a video is still useful; just tell me that the snapshot failed.
Do not count seconds or video frames yourself. I can help with that afterward.

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

1. Original CLOSE, MIDDLE and FAR shot videos.
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
CLOSE / MIDDLE / FAR:
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
FORWARD / BACKWARD / LEFT / RIGHT:
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
We finished close and middle shots. We did not get far shots or movement videos.
The CSV files are copied / still on the robot.
```

Prioritize useful stationary shot results first. Moving shots still need
integration and robot testing after we check the measurements.
