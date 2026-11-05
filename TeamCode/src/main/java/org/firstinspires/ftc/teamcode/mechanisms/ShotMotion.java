package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Moving-shot math for later integration, NOT permission to feed.
 * All offsets and velocities must use the SAME field axes (inches, seconds).
 * Assumes a fixed target and a projectile inheriting robot translation velocity.
 * Camera/shooter offsets, rotation, frame verification and measured flight/release
 * times must be handled before using this output to steer or change motor speed.
 */
public final class ShotMotion {
    private ShotMotion() { }

    public static final class Lead {
        public final double x;
        public final double y;
        public final double horizontalRange;
        public final double fieldBearingRadians;

        private Lead(double x, double y) {
            this.x = x;
            this.y = y;
            horizontalRange = Math.hypot(x, y);
            fieldBearingRadians = Math.atan2(y, x);
        }
    }

    public static Lead calculate(double targetOffsetX, double targetOffsetY,
                                 double velocityX, double velocityY,
                                 double flightSeconds, double releaseDelaySeconds) {
        if (!Double.isFinite(targetOffsetX) || !Double.isFinite(targetOffsetY)
                || !Double.isFinite(velocityX) || !Double.isFinite(velocityY)
                || !Double.isFinite(flightSeconds) || flightSeconds <= 0
                || !Double.isFinite(releaseDelaySeconds) || releaseDelaySeconds < 0)
            return null;
        double totalSeconds = flightSeconds + releaseDelaySeconds;
        double x = targetOffsetX - velocityX * totalSeconds;
        double y = targetOffsetY - velocityY * totalSeconds;
        if (!Double.isFinite(x) || !Double.isFinite(y)
                || !Double.isFinite(Math.hypot(x, y)) || Math.hypot(x, y) <= 0)
            return null;
        return new Lead(x, y);
    }
}
