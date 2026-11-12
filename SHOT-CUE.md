# Current shooting controls — Mimir_2026_Mec

As of November 11, TeleOp uses independent manual controls:

- Triangle / Y toggles the flywheel at the fixed 1325 ticks/s target.
- Circle / B toggles the indexer, including when the flywheel is off.
- Hold Square / X to reverse the indexer; release to resume its toggle state.
- Cross / A toggles the intake.

Shot prediction, taught spots, shooting arcs, camera processing, shot rumble,
and practice CSV recording are disconnected from this TeleOp. Existing autos,
alliance memory, drivetrain controls, and position hold are retained.

## Archived stationary shot cue experiment

The instructions below describe the disconnected experiment, not the current controls.

**Quick setup: score once, stop feeding, and press D-pad Up while still.** Leave
the shooter on and wait for **Flywheel ready**. Check **Teach spot** for
**Taught position for this run**. The controller rumbles when you are back at
that position and heading with the flywheel ready. This uses odometry and works
when the camera has no tag data. Teach the spot again after an OpMode restart.

**For new positions, teach a left shot and a right shot at roughly the same
distance from the same upward hive cell.** Keep the same shooter setting. Aim,
score, stop feeding and press Up at each spot, about two feet apart. Look for
**Estimated shooting arcs: 1** or higher. If it stays at zero, the two shots did
not provide a usable common-distance arc; the confirmed spots still work.

The robot uses the two rear-shooter directions to estimate where the target is.
It can then rumble at new positions **between those shooting angles**, within
their common distance band, when you manually aim the rear shooter toward that
estimated target. **Cue source: Estimated shooting arc** identifies this rough
geometric estimate. It does not predict shots at arbitrary closer/farther ranges
or change RPM. Check an intermediate shot before relying on the estimate.

The cue checks automatically after **START**. You aim, drive, turn the shooter
on, and feed manually. **Cue source** tells you which check matched. When full
tag data is available, Up also saves tag references that can survive a restart.

Recording starts with TeleOp. Use [SHOT-MATH-PRACTICE.md](SHOT-MATH-PRACTICE.md)
when you have time to collect measurements for prediction at new positions.

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
5. Wait for **Flywheel ready**. Use **Circle** to feed manually and confirm a
   ball goes into the hive. Pick a spot you can score from consistently.
6. Press **Circle** again to stop feeding. Leave the shooter on. Keep the robot
   in the same place and heading, and wait for its speed to recover.
7. Press and release **D-pad Up**. This means “I confirmed this position scores.”
8. Look for **Taught position for this run** under **Teach spot** on the Driver
   Station. If it says **Not taught**, follow the reason and press Up again.
9. After the robot has settled, the controller should rumble automatically.

You can teach more successful spots the same way. The odometry spots last until
STOP because each run can start with a different odometry origin. Full tag
references, when available at the time you press Up, are saved across restarts.

## What turns the rumble off?

- Moving a drive/turn stick, or actual robot movement above the stillness limit.
- Leaving the taught position or turning away from its heading.
- For an arc estimate, leaving its learned angle/range band or aiming away from
  the estimated target.
- For a tag match, losing fresh 3D tag data or leaving the saved tag window.
- Turning the shooter off, changing its target setting, or a speed drop.

Manual feeding still works when the indicator is quiet. Rumble means the
tested measurement checks passed; shot outcome also depends on ball delivery
and the shooter mechanism.

## If it does not rumble

Read **SHOT CUE** near the top of the Driver Station. It gives the current
reason. **Teach spot** reports whether teaching worked. The same screen shows
tag readings, actual robot motion, and target/actual flywheel speed.

The taught position and saved tag methods match a small area around a confirmed
shot. The arc method estimates additional positions at a common distance between
two confirmed shooting angles. Prediction across a broader distance range needs
verified field geometry and launch measurements. All methods require stationary
shots and a recovered flywheel.

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
- Same alliance and shooter target (within 1 tick/second).
- Taught position: within 2 inches and 2 degrees of the confirmed pose.
- Tag reference: same tag ID, TX and TY within 1.5 degrees, 3D range within 2
  inches, and tag orientation within 6 degrees per axis.
- Existing flywheel recovery check: 97–103% of target for 0.25 seconds.

These are starting matching windows to verify at practice. No default
real-hive scoring position is preloaded.
