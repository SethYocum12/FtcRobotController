# Stationary shot rumble — Mimir_2026_Mec

For the next practice, use [SHOT-MATH-PRACTICE.md](SHOT-MATH-PRACTICE.md):
**4 individual shots at each of five positions** for an initial fit. Extend
LEFT/RIGHT to ten each when time allows to check consistency. Practice recording
saves the readings needed to fit and validate the new flight model.

After **START**, the cue checks automatically. A validated flight model checks
the predicted path through the opening at new positions inside its tested
distance range. Until the model is trained, feedback uses confirmed saved spots.
**Cue source** on the Driver Station identifies which method is active.
You aim, drive, turn the shooter on, and feed the balls yourself.

The indicator starts with **no confirmed shots**. It does not treat old home
videos or tag detection alone as a working real-hive shot.

## Official hive dimensions checked on November 8

FIRST's [Arena TU06 drawings](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09)
show the following nominal dimensions (Figures 9-10, 9-11 and 9-15):

| Measurement | Inches |
| --- | ---: |
| Tag black square side | 3.25 |
| Cell opening width | 20 |
| Opening face height at its peak | 14 |
| Lowest edge of the upward opening above the tiles | 53.5 |
| Highest edge of that opening above the tiles | 65.6 |

The face is pentagonal and tilted; its 14-inch face height is different from
its vertical floor-height span. The camera's existing **82.55 mm / 3.25 inch**
marker-size setting matches the real tags, so its scale needs no change.

Your home board's top was **54.5 inches**, one inch above the real lower lip.
Those shots establish clearance over that edge. We do not have a verified
ball-height/landing envelope proving they also fit beneath the real upper rim.
Opening dimensions alone cannot supply that missing shooter measurement.

The official dimensions are recorded in the saved reference metadata and
[home-to-field comparison](practice-analysis/2026-11-08/real-hive-geometry.json).
Readiness still uses confirmed scoring references. This update starts a separate
real-hive reference file, preserving the older file on the Hub.

## Save your first successful spot

1. Upload this code and select **Mimir_2026_Mec** on the Driver Station.
2. Press **INIT**. Check the alliance. D-pad Left selects red; Right selects blue.
3. Press **START**. Drive to a spot at the real hive and aim by hand.
4. Release the drive sticks. Press **Triangle** to turn the shooter on.
5. Wait for the flywheel to reach speed. Use **Circle** to feed manually.
   Confirm that several shots from this spot actually go into the hive.
6. Press **Circle** again to stop feeding. Leave the shooter on. Keep the robot
   in the same place and heading, and wait for its speed to recover.
7. Press and release **D-pad Up**. This means “I confirmed this position scores.”
8. Look for **Saved confirmed position** on the Driver Station. If it says
   **Not saved**, follow the reason on that line and press Up again.
9. After the robot has settled, the controller should rumble automatically.

You can save additional successful spots the same way. Each visible alliance
tag gets its own reference from that spot. Different tag IDs are never treated
as the same point. References survive STOP, OpMode changes and normal restarts.

## What turns the rumble off?

- Moving a drive/turn stick, or actual robot movement above the stillness limit.
- Losing a fresh alliance tag or its full 3D measurement.
- Moving outside the saved angle, range or approach window.
- Turning the shooter off, changing its target setting, or a speed drop.

Manual feeding still works when the indicator is quiet. Rumble means the
tested measurement checks passed; shot outcome also depends on ball delivery
and the shooter mechanism.

## If it does not rumble

Read **SHOT CUE** near the top of the Driver Station. It gives the current
reason. **Saved shots** reports whether saving worked. The same screen shows
tag readings, actual robot motion, and target/actual flywheel speed.

The saved-spot method uses a narrow neighborhood of each confirmed shot.
The trajectory method generalizes within its fitted distance range after field
geometry, launch parameters and separate validation shots are verified.
Both methods require stationary shots and a recovered flywheel.

## Clear outdated references

After changing the camera mounting, shooter geometry, tag size/pipeline
calibration, or switching from a home target to a real hive, clear the old
references and confirm fresh scoring positions.

During **INIT**, hold **Square + Triangle together for one second**.
Look for **Cleared confirmed shot references** and **Math calibration cleared**.
This clears shot references and disables the fitted model until recalibrated.

The Control Hub file is
`FIRST/settings/mimir-biobuzz-hive-shot-cue.properties`.
The flight model uses `FIRST/settings/mimir-stationary-shot-math.properties`.
Practice CSV sessions are under `FIRST/settings/mimir-shot-math/`.

## Current starting windows

- Still for 0.4 seconds, with centered drive sticks.
- Measured translation at most 1 inch/second; turn at most 3 degrees/second.
- Same tag ID, alliance, and shooter target (within 1 tick/second).
- TX and TY each within 1.5 degrees of the confirmed reading.
- Camera-to-tag 3D range within 2 inches; tag orientation within 6 degrees per axis.
- Existing flywheel recovery check: 97–103% of target for 0.25 seconds.

These are starting matching windows to verify at practice. No default
real-hive scoring position is preloaded.
