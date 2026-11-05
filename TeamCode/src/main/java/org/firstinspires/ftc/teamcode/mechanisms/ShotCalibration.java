package org.firstinspires.ftc.teamcode.mechanisms;

public class ShotCalibration {
    // Leave disabled until tape checks and stationary scoring trials pass.
    // Distances below are CAMERA-TO-TAG slant ranges, matching AimAssist.
    public static final boolean ENABLE_STATIONARY_SMART_SHOTS = false;
    public static final boolean RANGE_VERIFIED = false;
    public static final boolean AIM_VERIFIED = false;
    public static final int CALIBRATED_TAG_ID = -1;
    public static final double AIM_OFFSET_DEGREES = 0.0;
    public static final double[][] SHOTS = {

    };

    public  static boolean hasEnoughSamples() {
        return SHOTS.length >= 2;

    }

    public static boolean isValidTable() {
        return isValidTable(SHOTS);
    }

    static boolean isValidTable(double[][] samples) {
        if (samples == null || samples.length < 2) return false;
        double previous = 0;
        for (double[] row : samples) {
            if (row == null || row.length != 2
                    || !Double.isFinite(row[0]) || row[0] <= previous
                    || !Double.isFinite(row[1]) || row[1] <= 0) return false;
            previous = row[0];
        }
        return true;
    }

    public static boolean isDistanceInRange(double distanceInches) {
        if (!isValidTable() || !Double.isFinite(distanceInches)) {
            return false;
        }

        double nearest = SHOTS[0][0];
        double farthest = SHOTS[SHOTS.length - 1][0];

        return distanceInches >= nearest
                && distanceInches <= farthest;
    }

    public static double velocityFor(double distanceInches) {
        return velocityFor(SHOTS, distanceInches);
    }

    static double velocityFor(double[][] samples, double distanceInches) {
        if (!isValidTable(samples) || !Double.isFinite(distanceInches)
                || distanceInches < samples[0][0]
                || distanceInches > samples[samples.length - 1][0]) {
            return Double.NaN;
        }

        for (int i = 1; i < samples.length; i++) {
            double nearDistance = samples[i - 1][0];
            double farDistance = samples[i][0];

            if (distanceInches <= farDistance) {
                double nearVelocity = samples[i - 1][1];
                double farVelocity = samples[i][1];

                double fraction = (distanceInches - nearDistance)
                        / (farDistance - nearDistance);

                return nearVelocity
                        + fraction * (farVelocity - nearVelocity);
            }
        }

        return Double.NaN;
    }

}
