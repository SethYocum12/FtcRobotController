# Shooting autonomous policy

Seth's requested default for current and future shooting paths:

- If a new export conflicts with constraints established in this conversation,
  correct the conflict and tell Seth exactly what changed. Otherwise preserve
  the exported geometry and headings that already meet those constraints.
- Measure time from Driver Station START. Reset the OpMode's `ElapsedTime`
  immediately after `waitForStart()` and the stop-request check.
- After START, call `shooting.rememberAlliance()` before scheduling the routine.
  It saves this auto's red/blue selection in `AllianceMemory` for the quick
  transition into `Mimir_2026_Mec`. Merely scanning or initializing an auto must
  not change the saved alliance. Memory lasts until the app process restarts.
- Follow path 1, hold its shooting pose, and spin up the shooter.
- October 31 adjustments: path 1 shooting endpoints were moved toward the
  starting wall in two requested 5-inch steps, with shooting Y limited to
  11..130.5 so the centered 18-inch footprint retains a 2-inch wall gap.
  Path 2 starts match the revised shooting poses. Some second moves are shorter
  than 5 inches because of that clearance limit. A subsequent explicit route
  replacement takes its shooting coordinates from the new export when they
  satisfy the footprint constraints; do not repeatedly apply old relative moves.
- October 31 Blue No Gar replacement: `BlueNoGarShootParkLWWR` replaces the old
  top-wall No Gar route. Its new export starts at (79.2181, 8.324, 270), shoots
  at (83.1011, 13.0302, 270), and parks at (126.0291, 19.4637, 180).
- Start feeding as soon as a fresh AprilTag from one unambiguous alliance cell
  is visible and measured shooter velocity reaches 95% of the target. There is
  no fixed delay and no separate readiness timeout.
- Once triggered, keep feeding until 20 seconds after START. Tag loss does not
  end feeding, reselect a cell, or restart the feeding timer.
- Stop both shooter and indexer and begin the second path at the 20-second
  cutoff, including when no tag appeared, shooter speed was never reached, or
  the first path did not finish. The normal OpMode loop processes the cutoff.
- Indexer recovery: reverse for 0.7 seconds starting at 3, 6, 9, ... seconds
  after feeding begins, then resume forward. Repeat every 3 seconds until the
  global cutoff; there is no longer a three-pulse limit. The clock begins when
  feeding starts and does not restart when a tag disappears.
- STOP/cancellation also shuts down the shooter and indexer.
- Preserve each exported path and use unique `@Autonomous` display names.
- Use Seth's L/R labels for named routes. The Blue Gar route ending near
  (126.5, 51) is park R, as clarified on October 30; do not relabel it L based
  on a different driver or field-map viewpoint.
- Starting orientation: the rear-mounted shooter faces into the field and the
  front faces the starting wall. For the current bottom-wall starts use 270
  degrees; for top-wall starts use 90 degrees. Use the same heading at the
  shooting pose and the beginning of the second path. Keep exported X/Y geometry.
- Parking orientation: FRONT toward the hive side, as clarified by Seth.
  Current left-wall red parking endpoints use 0 degrees (right on the field map);
  right-wall blue endpoints use 180 degrees (left on the field map).
  The rear shooter faces the field during the earlier shooting stage only.
- Parking clearance: Seth confirmed an 18-by-18-inch footprint. Assuming the
  Pedro pose is at its center, keep the final parked footprint at least 2 inches
  from every wall. Use the 141.5-inch inside field coordinate boundary. At
  cardinal headings this means center coordinates between 11 and 130.5 inches;
  at other headings account for the rotated square's larger projected extent.
  Move only final parking endpoints that are too close; preserve larger gaps.
  This overrides exported endpoint coordinates when needed. It does not add
  clearance to initial wall-contact poses or guarantee clearance along a curve.

Use `mechanisms/AutonomousShooting.java` for this behavior instead of copying the
old 3/16-second readiness waits, 7/10-second feed limits, or tag-loss stop logic.
All current shooting autos use it, including `RedGarShootParkRWWR`. Pass `true`
for a red auto, `false` for blue. Older deleted autos are not automatically restored.

## Using it in a new path export

Create a shared helper after the mechanism and vision fields:

```java
private final ElapsedTime autoTimer = new ElapsedTime();
private final AutonomousShooting shooting = new AutonomousShooting(
        mechanisms, vision, autoTimer::seconds, false,
        SHOOTER_TARGET_VELOCITY, INDEXER_POWER);
```

The routine begins with:

```java
return sequential(
        shooting.approachAndShoot(follower, path1(), path1),
        follow(follower, path2())
);
```

Reset `autoTimer`, call `shooting.rememberAlliance()`, and then schedule the
routine after START and the stop-request check.
Continue updating vision before `Scheduler.execute()`, and continue calling
`follower.update()` each loop. Call `shooting.stop()` in the OpMode's `finally`
block, together with the existing follower, vision, and dashboard cleanup.
Any intake/collection actions in later paths still need their own cleanup.

The policy changes sequencing and mechanism control, not the drive geometry.
The remaining path must still be tested to finish within the autonomous period.
