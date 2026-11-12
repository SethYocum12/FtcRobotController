package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.ArrayList;
import java.util.List;

/** Confirmed spots and an estimated shooting arc in this run's odometry frame only. */
public final class ConfirmedShotPoseCue {
    public static final double POSITION_WINDOW_INCHES = 2.0;
    public static final double HEADING_WINDOW_RADIANS = Math.toRadians(2.0);
    private static final double TARGET_WINDOW_TICKS_PER_SECOND = 1.0;
    private static final int MAX_POSITIONS = 16;
    private static final double MIN_BASELINE_INCHES = 12.0;
    private static final double MIN_ARC_RADIANS = Math.toRadians(15.0);
    private static final double MAX_ARC_RADIANS = Math.toRadians(45.0);

    private static final class Reference {
        final double x, y, heading, velocity;
        final boolean red;
        final int cell;

        Reference(StationaryShotCue.Reading r) {
            x = r.robotX;
            y = r.robotY;
            heading = r.robotHeading;
            velocity = r.targetVelocity;
            red = r.redAlliance;
            int visibleCell = -1;
            for (StationaryShotCue.Target target : r.targets) {
                if (target.id < (red ? 30 : 38) || target.id > (red ? 37 : 45)) continue;
                int id = (target.id - 30) / 4;
                if (visibleCell >= 0 && visibleCell != id) { visibleCell = -1; break; }
                visibleCell = id;
            }
            cell = visibleCell;
        }
    }

    /** Two rear-shooter aim rays estimate the target; their common range band defines the arc. */
    private static final class Arc {
        final Reference first;
        final double targetX, targetY, minRadius, maxRadius, startBearing, sweep;

        Arc(Reference first, double x, double y, double min, double max, double start, double sweep) {
            this.first = first;
            targetX = x; targetY = y;
            minRadius = min; maxRadius = max;
            startBearing = start; this.sweep = sweep;
        }

        static Arc between(Reference a, Reference b) {
            if (a.red != b.red || Math.abs(a.velocity - b.velocity) > TARGET_WINDOW_TICKS_PER_SECOND
                    || a.cell >= 0 && b.cell >= 0 && a.cell != b.cell
                    || Math.hypot(b.x - a.x, b.y - a.y) < MIN_BASELINE_INCHES) return null;
            double turn = angleDistance(a.heading, b.heading);
            if (turn < MIN_ARC_RADIANS || turn > MAX_ARC_RADIANS) return null;
            // Heading describes the front. The shooter fires from the rear.
            double ax = -Math.cos(a.heading), ay = -Math.sin(a.heading);
            double bx = -Math.cos(b.heading), by = -Math.sin(b.heading);
            double cross = ax * by - ay * bx;
            double dx = b.x - a.x, dy = b.y - a.y;
            double distanceA = (dx * by - dy * bx) / cross;
            double distanceB = (dx * ay - dy * ax) / cross;
            if (!Double.isFinite(distanceA) || !Double.isFinite(distanceB)
                    || distanceA < 18 || distanceB < 18 || distanceA > 200 || distanceB > 200) return null;
            double min = Math.max(distanceA, distanceB) - POSITION_WINDOW_INCHES;
            double max = Math.min(distanceA, distanceB) + POSITION_WINDOW_INCHES;
            // Both successful shots must support the new radius; never bridge a wide distance gap.
            if (max - min < 0.5) return null;
            double x = a.x + distanceA * ax, y = a.y + distanceA * ay;
            double start = Math.atan2(a.y - y, a.x - x);
            double sweep = wrap(Math.atan2(b.y - y, b.x - x) - start);
            return new Arc(a, x, y, min, max, start, sweep);
        }

        boolean matches(StationaryShotCue.Reading r) {
            if (first.red != r.redAlliance
                    || Math.abs(first.velocity - r.targetVelocity) > TARGET_WINDOW_TICKS_PER_SECOND) return false;
            double dx = r.robotX - targetX, dy = r.robotY - targetY;
            double radius = Math.hypot(dx, dy);
            if (radius < minRadius || radius > maxRadius) return false;
            double bearing = Math.atan2(dy, dx);
            double offset = wrap(bearing - startBearing);
            if (sweep > 0 ? offset < -1e-9 || offset > sweep + 1e-9
                    : offset > 1e-9 || offset < sweep - 1e-9) return false;
            // A rear-mounted shooter aims toward the target when the front points outward.
            return angleDistance(r.robotHeading, bearing) <= HEADING_WINDOW_RADIANS;
        }
    }

    private final List<Reference> references = new ArrayList<>();
    private final List<Arc> arcs = new ArrayList<>();
    private double lastTime = Double.NaN;
    private boolean stoppedAtSpeed;
    private boolean usingArcEstimate;
    private String status = "After scoring: stop feeding, then Up teaches this spot";
    private String saveStatus = "No position taught this run";

    /** Never carry an odometry pose through an OpMode restart or a new origin. */
    public void reset() {
        references.clear();
        arcs.clear();
        usingArcEstimate = false;
        lastTime = Double.NaN;
        stoppedAtSpeed = false;
        status = "After scoring: stop feeding, then Up teaches this spot";
        saveStatus = "No position taught this run";
    }

    public boolean update(StationaryShotCue.Reading r, boolean stoppedAtSpeed, String reason) {
        lastTime = r.seconds;
        usingArcEstimate = false;
        this.stoppedAtSpeed = stoppedAtSpeed;
        if (!stoppedAtSpeed) {
            status = reason == null ? "Wait for stationary shooter checks" : reason;
            return false;
        }
        if (!validPose(r)) {
            status = "Need valid odometry pose";
            return false;
        }
        for (Reference ref : references) {
            if (ref.red == r.redAlliance
                    && Math.abs(ref.velocity - r.targetVelocity) <= TARGET_WINDOW_TICKS_PER_SECOND
                    && Math.hypot(ref.x - r.robotX, ref.y - r.robotY) <= POSITION_WINDOW_INCHES
                    && angleDistance(ref.heading, r.robotHeading) <= HEADING_WINDOW_RADIANS) {
                status = "Matches confirmed position and heading (this run)";
                return true;
            }
        }
        for (Arc arc : arcs) {
            if (arc.matches(r)) {
                usingArcEstimate = true;
                status = "Estimated arc: range and rear-shooter aim match";
                return true;
            }
        }
        status = references.isEmpty()
                ? "After scoring: stop feeding, then Up teaches this spot"
                : arcs.isEmpty() ? "Teach a second scoring angle at the same hive/distance for an arc"
                : "Match learned arc distance and rear-shooter aim; stop to rumble";
        return false;
    }

    /** Up is the driver's confirmation that a real shot went in, not an inferred basket. */
    public boolean confirm(StationaryShotCue.Reading r) {
        if (!Double.isFinite(r.seconds) || Double.compare(lastTime, r.seconds) != 0
                || !stoppedAtSpeed || !validPose(r)) {
            saveStatus = "Not taught: " + status;
            return false;
        }
        if (r.feeding || r.reversing) {
            saveStatus = "Stop feeding/reversing, leave shooter on, then press Up";
            return false;
        }
        Reference replacement = new Reference(r);
        for (int i = 0; i < references.size(); i++) {
            Reference old = references.get(i);
            if (old.red == r.redAlliance
                    && Math.abs(old.velocity - r.targetVelocity) <= TARGET_WINDOW_TICKS_PER_SECOND
                    && Math.hypot(old.x - r.robotX, old.y - r.robotY) < 0.5
                    && angleDistance(old.heading, r.robotHeading) < Math.toRadians(1)) {
                references.set(i, replacement);
                rebuildArcs();
                return true;
            }
        }
        if (references.size() >= MAX_POSITIONS) {
            saveStatus = "Not taught: position limit reached";
            return false;
        }
        references.add(replacement);
        rebuildArcs();
        return true;
    }

    private void rebuildArcs() {
        arcs.clear();
        for (int i = 0; i < references.size(); i++) {
            for (int j = i + 1; j < references.size(); j++) {
                Arc arc = Arc.between(references.get(i), references.get(j));
                if (arc != null) arcs.add(arc);
            }
        }
        saveStatus = arcs.isEmpty() ? "Taught position for this run; teach a second scoring angle for an arc"
                : "Taught position; estimated arc available between scoring angles";
    }

    private static boolean validPose(StationaryShotCue.Reading r) {
        return r.allianceKnown && r.shooterOn && r.flywheelReady
                && Double.isFinite(r.seconds) && Double.isFinite(r.targetVelocity) && r.targetVelocity > 0
                && Double.isFinite(r.robotX) && Double.isFinite(r.robotY) && Double.isFinite(r.robotHeading);
    }

    private static double angleDistance(double first, double second) {
        return Math.abs(wrap(first - second));
    }

    private static double wrap(double angle) { return Math.IEEEremainder(angle, 2.0 * Math.PI); }

    public int getReferenceCount() { return references.size(); }
    public int getArcCount() { return arcs.size(); }
    public boolean isUsingArcEstimate() { return usingArcEstimate; }
    public String getStatus() { return status; }
    public String getSaveStatus() { return saveStatus; }
}
