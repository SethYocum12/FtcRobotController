package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Fixed-speed, constant-velocity lead in a measured corridor in Pedro field axes. */
public class SidewaysShotPlanner {
    public static final Calibration CALIBRATION = new Calibration();

    /** NaN values require measurements. Limits are initial test limits, not measured data. */
    public static class Calibration {
        public boolean enabled = false;
        public boolean geometryAndTimingVerified = false;
        public int tagId = -1;
        public double fixedVelocity = Double.NaN;
        public double flightSeconds = Double.NaN;
        public double releaseDelaySeconds = Double.NaN;
        // All field positions share the EXACT origin/heading used by TeleOp Pinpoint.
        public double targetX = Double.NaN, targetY = Double.NaN;
        public double lineX = Double.NaN, lineY = Double.NaN;
        // Direction along the sideways line, CCW from field +X.
        public double lineHeadingDegrees = Double.NaN;
        public double minAlong = Double.NaN, maxAlong = Double.NaN;
        public double shooterForward = Double.NaN, shooterLeft = Double.NaN;
        // Successful stationary raw TY at the reference point, verified across the corridor.
        public double aimOffsetDegrees = Double.NaN;
        // +1 if a CCW target-bearing change increases TY, otherwise -1. Must be tested.
        public double tyPerCcwDegree = Double.NaN;
        public double maxSidewaysSpeed = 6.0;
        public double maxNormalSpeed = 1.0;
        public double maxOffLine = 1.0;
        public double maxTurnDegreesPerSecond = 5.0;
        public double maxAcceleration = 6.0;
        public double stableSeconds = 0.2;
        public double aimToleranceDegrees = 2.0;
        public double maxVisionAgeMs = 100.0;

        public String problem() {
            if (!enabled) return "Sideways shots disabled";
            if (!geometryAndTimingVerified) return "Need measured sideways calibration";
            double[] numbers = {fixedVelocity, flightSeconds, releaseDelaySeconds,
                    targetX, targetY, lineX, lineY, lineHeadingDegrees, minAlong, maxAlong,
                    shooterForward, shooterLeft, aimOffsetDegrees, tyPerCcwDegree,
                    maxSidewaysSpeed, maxNormalSpeed, maxOffLine, maxTurnDegreesPerSecond,
                    maxAcceleration, stableSeconds, aimToleranceDegrees, maxVisionAgeMs};
            for (double value : numbers)
                if (!Double.isFinite(value)) return "Sideways measurements incomplete";
            if (!(tagId >= 30 && tagId <= 45) || fixedVelocity <= 0 || flightSeconds <= 0
                    || releaseDelaySeconds < 0 || minAlong >= maxAlong
                    || Math.abs(tyPerCcwDegree) != 1 || maxSidewaysSpeed <= 0
                    || maxNormalSpeed < 0 || maxOffLine <= 0 || maxTurnDegreesPerSecond <= 0
                    || maxAcceleration <= 0 || stableSeconds <= 0
                    || aimToleranceDegrees <= 0 || maxVisionAgeMs <= 0 || maxVisionAgeMs > 250)
                return "Invalid sideways calibration";
            return null;
        }
    }

    private final AimAssist aim;
    private final servos_and_shooters shooter;
    private final Calibration calibration;
    private final ElapsedTime clock = new ElapsedTime();
    private double previousTime = Double.NaN, previousVx, previousVy, stableSince;
    private boolean stable;

    public SidewaysShotPlanner(AimAssist aim, servos_and_shooters shooter) {
        this(aim, shooter, CALIBRATION);
    }

    public SidewaysShotPlanner(AimAssist aim, servos_and_shooters shooter, Calibration calibration) {
        this.aim = aim;
        this.shooter = shooter;
        this.calibration = calibration;
    }

    public void reset() {
        previousTime = Double.NaN;
        stable = false;
    }

    private ShotSolution blocked(String status) {
        reset();
        return new ShotSolution(0, calibration.fixedVelocity, false, status);
    }

    public ShotSolution evaluate(boolean red, Pose pose, Velocity velocity) {
        Calibration c = calibration;
        String problem = c.problem();
        if (problem != null) return blocked(problem);
        if (!aim.hasAimTarget(red) || aim.getAimTagId(red) != c.tagId)
            return blocked("Need fresh calibrated alliance tag");
        long ageMs = aim.getAimAgeMs();
        double rawError = aim.getRawAimError(red);
        double range = aim.getTagRangeInches(red);
        if (ageMs < 0 || ageMs > c.maxVisionAgeMs || !Double.isFinite(rawError)
                || !Double.isFinite(range) || range <= 0)
            return blocked("Need fresh tag angle and camera range");
        if (pose == null || velocity == null || !Double.isFinite(pose.x())
                || !Double.isFinite(pose.y()) || !Double.isFinite(pose.heading())
                || !Double.isFinite(velocity.vx) || !Double.isFinite(velocity.vy)
                || !Double.isFinite(velocity.omega)) return blocked("Invalid odometry");

        double line = Math.toRadians(c.lineHeadingDegrees);
        double lx = Math.cos(line), ly = Math.sin(line);
        double px = pose.x() - c.lineX, py = pose.y() - c.lineY;
        double along = px * lx + py * ly;
        double cross = -px * ly + py * lx;
        double sideSpeed = velocity.vx * lx + velocity.vy * ly;
        double normalSpeed = -velocity.vx * ly + velocity.vy * lx;
        double predictedAlong = along + sideSpeed * c.releaseDelaySeconds;
        double predictedCross = cross + normalSpeed * c.releaseDelaySeconds;
        if (along < c.minAlong || along > c.maxAlong || Math.abs(cross) > c.maxOffLine
                || predictedAlong < c.minAlong || predictedAlong > c.maxAlong
                || Math.abs(predictedCross) > c.maxOffLine)
            return blocked("Outside measured shooting line");

        double heading = pose.heading();
        double offsetX = c.shooterForward * Math.cos(heading) - c.shooterLeft * Math.sin(heading);
        double offsetY = c.shooterForward * Math.sin(heading) + c.shooterLeft * Math.cos(heading);
        double dx = c.targetX - (pose.x() + offsetX);
        double dy = c.targetY - (pose.y() + offsetY);
        // Include tangential velocity at an off-center shooter.
        double vx = velocity.vx - velocity.omega * offsetY;
        double vy = velocity.vy + velocity.omega * offsetX;
        // This aim is recomputed from the CURRENT pose every loop, including while
        // indexing. Add only airborne inherited travel here. Adding feed delay
        // too would repeatedly predict that travel and over-lead at release.
        // Feed delay still bounds the predicted release position in the corridor above.
        ShotMotion.Lead lead = ShotMotion.calculate(dx, dy, vx, vy,
                c.flightSeconds, 0);
        if (lead == null || Math.hypot(dx, dy) <= 0) return blocked("Invalid target geometry");
        double bearing = Math.atan2(dy, dx);
        double leadDegrees = Math.toDegrees(wrap(lead.fieldBearingRadians - bearing)) * c.tyPerCcwDegree;
        // Approximate observation-age correction at the bounded, steady test speeds.
        double age = ageMs / 1000.0;
        double oldBearing = Math.atan2(dy + vy * age, dx + vx * age);
        double ageCorrection = Math.toDegrees(wrap(bearing - oldBearing)
                - velocity.omega * age) * c.tyPerCcwDegree;
        double error = rawError - c.aimOffsetDegrees + leadDegrees + ageCorrection;
        if (!Double.isFinite(error)) return blocked("Invalid compensated aim");
        double turn = aim.calculateTurn(error);

        double now = clock.seconds();
        double dt = now - previousTime;
        boolean steady = Double.isFinite(previousTime) && dt > 0 && dt <= 0.25
                && Math.hypot(velocity.vx - previousVx, velocity.vy - previousVy) / dt <= c.maxAcceleration;
        previousTime = now;
        previousVx = velocity.vx;
        previousVy = velocity.vy;
        boolean movementAllowed = Math.abs(sideSpeed) <= c.maxSidewaysSpeed
                && Math.abs(normalSpeed) <= c.maxNormalSpeed
                && Math.abs(Math.toDegrees(velocity.omega)) <= c.maxTurnDegreesPerSecond;
        if (!steady || !movementAllowed) stable = false;
        else if (!stable) { stable = true; stableSince = now; }
        boolean ready = false;
        String status;
        double maximumVelocity = shooter.getMaxShooterVelocity();
        if (!Double.isFinite(maximumVelocity) || maximumVelocity <= 0
                || c.fixedVelocity > maximumVelocity) status = "Fixed speed exceeds motor limit";
        else if (!shooter.isShooterEnabled()) status = "Shooter off";
        else if (!Double.isFinite(shooter.getShooterTargetVelocity())
                || Math.abs(shooter.getShooterTargetVelocity() - c.fixedVelocity) > 0.5)
            status = "Waiting for fixed flywheel target";
        else if (!Double.isFinite(shooter.getShooterVelocity())
                || shooter.getShooterVelocity() < c.fixedVelocity * 0.97
                || shooter.getShooterVelocity() > c.fixedVelocity * 1.03
                || !shooter.isShooterReady())
            status = "Waiting for stable flywheel speed / recovery";
        else if (!movementAllowed) status = "Move sideways slowly; stay on line";
        else if (Math.abs(error) > c.aimToleranceDegrees) status = "Aiming with sideways lead";
        else if (!stable || now - stableSince < c.stableSeconds) status = "Wait for steady movement";
        else { ready = true; status = "Sideways shot ready"; }
        return new ShotSolution(turn, c.fixedVelocity, ready, status,
                error, leadDegrees, sideSpeed, normalSpeed);
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
