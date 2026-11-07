package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AimAssist {

    private static final double TURN_GAIN = 0.01;
    private static final double MAX_TURN = 0.15;
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
    private static final double TRACKING_TURN_MULTIPLIER = 1.5;
    private static final double MAX_TRACKING_TURN = 0.22;
    private final LIMELIGHT_setup vision;

    public AimAssist(LIMELIGHT_setup vision) {
        this.vision = vision;
    }


    public boolean hasAimTarget(boolean redAlliance) {
        return Double.isFinite(getAimError(redAlliance));
    }

    public double getTurn(boolean redAlliance) {
        double error = getAimError(redAlliance);

        if (!Double.isFinite(error)) {
            return 0;
        }

        return calculateTurn(error);
    }

    public boolean isAligned(boolean redAlliance) {
        double error = getAimError(redAlliance);

        return Double.isFinite(error)
                && Math.abs(error) <= AIM_TOLERANCE_DEGREES;
    }

    public double calculateTurn(double errorDegrees) {
        if (Math.abs(errorDegrees) <= AIM_TOLERANCE_DEGREES) {
            return 0;
        }

        double turn = errorDegrees * TURN_GAIN;
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));
    }

    /** Faster tracking with B OFF; retain the normal correction when B requests feed. */
    public double trackingTurn(double plannedTurn, boolean feedRequested) {
        if (!Double.isFinite(plannedTurn)) return 0;
        if (feedRequested) return Math.max(-MAX_TURN, Math.min(MAX_TURN, plannedTurn));
        return Math.max(-MAX_TRACKING_TURN,
                Math.min(MAX_TRACKING_TURN, plannedTurn * TRACKING_TURN_MULTIPLIER));
    }

    public double getTagRangeInches(boolean redAlliance) {
        return getTagRangeReading(redAlliance).inches;
    }

    public static final class RangeReading {
        public final double inches;
        public final String status;

        private RangeReading(double inches, String status) {
            this.inches = inches;
            this.status = status;
        }
    }

    public RangeReading getTagRangeReading(boolean redAlliance) {
        LLResultTypes.FiducialResult tag = vision.getAimTag(redAlliance);
        if (tag == null) {
            return new RangeReading(Double.NaN, "No fresh alliance tag");
        }

        double direct = poseRangeInches(tag.getTargetPoseCameraSpace());
        if (Double.isFinite(direct)) {
            return new RangeReading(direct, "Distance OK: target-to-camera data");
        }

        // These are inverse transforms of the SAME camera/tag pair, so their
        // translation lengths are equal. Never use robot pose or another tag.
        // Fields: https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification
        double inverse = poseRangeInches(tag.getCameraPoseTargetSpace());
        if (Double.isFinite(inverse)) {
            return new RangeReading(inverse, "Distance OK: camera-to-target data");
        }
        return new RangeReading(Double.NaN,
                "Tag seen, no 3D distance: check pipeline 8 Full 3D");
    }

    private static double poseRangeInches(Pose3D pose) {
        if (pose == null || pose.getPosition() == null) return Double.NaN;
        Position position = pose.getPosition().toUnit(DistanceUnit.INCH);

        double range = Math.hypot(Math.hypot(position.x, position.y), position.z);

        return Double.isFinite(range) && range > 0
                ? range : Double.NaN;
    }

    public double getAimError(boolean redAlliance) {
        double raw = getRawAimError(redAlliance);
        return ShotCalibration.AIM_VERIFIED
                && getAimTagId(redAlliance) == ShotCalibration.CALIBRATED_TAG_ID
                ? raw - ShotCalibration.AIM_OFFSET_DEGREES : raw;
    }

    public double getRawAimError(boolean redAlliance) {
        LLResultTypes.FiducialResult tag = vision.getAimTag(redAlliance);

        if (tag == null) {
            return Double.NaN;
        }

        return tag.getTargetYDegrees();
    }

    public int getAimTagId(boolean redAlliance) {
        LLResultTypes.FiducialResult tag = vision.getAimTag(redAlliance);

        return tag == null ? -1 : tag.getFiducialId();
    }

    public long getAimAgeMs() { return vision.getResultAgeMs(); }
}
