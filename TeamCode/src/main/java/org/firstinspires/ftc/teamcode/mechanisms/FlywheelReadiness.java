package org.firstinspires.ftc.teamcode.mechanisms;

/** Speed permission only; this does not count balls or prove a shot will score. */
public final class FlywheelReadiness {
    public static final double MIN_SPEED_RATIO = 0.97;
    public static final double MAX_SPEED_RATIO = 1.03;
    public static final double SETTLE_SECONDS = 0.25;
    private static final double MAX_SAMPLE_GAP_SECONDS = 0.25;

    private double previousTime = Double.NaN;
    private double previousTarget = Double.NaN;
    private double inBandSince = Double.NaN;
    private boolean ready;
    private int speedDrops;
    private String status = "Shooter off";

    public boolean update(double seconds, boolean enabled, double target, double actual) {
        boolean valid = Double.isFinite(seconds) && Double.isFinite(target) && target > 0
                && Double.isFinite(actual) && actual >= 0;
        if (!enabled || !valid) {
            ready = false;
            previousTime = Double.NaN;
            previousTarget = Double.NaN;
            inBandSince = Double.NaN;
            status = !enabled ? "Shooter off" : "Invalid flywheel reading";
            return false;
        }

        boolean changedTarget = Double.compare(target, previousTarget) != 0;
        boolean brokenTiming = !Double.isFinite(previousTime) || seconds < previousTime
                || seconds - previousTime > MAX_SAMPLE_GAP_SECONDS;
        if (changedTarget || brokenTiming) {
            ready = false;
            inBandSince = Double.NaN;
        }
        previousTarget = target;
        previousTime = seconds;
        if (actual < target * MIN_SPEED_RATIO || actual > target * MAX_SPEED_RATIO) {
            if (ready && actual < target * MIN_SPEED_RATIO) speedDrops++;
            ready = false;
            inBandSince = Double.NaN;
            status = actual < target * MIN_SPEED_RATIO
                    ? "Waiting for 97% speed / recovery" : "Waiting for speed to come down";
            return false;
        }
        if (!Double.isFinite(inBandSince)) inBandSince = seconds;
        ready = seconds - inBandSince >= SETTLE_SECONDS;
        status = ready ? "Flywheel ready" : "Speed OK; settling for 0.25 s";
        return ready;
    }

    public String getStatus() { return status; }
    public int getSpeedDrops() { return speedDrops; }
}
