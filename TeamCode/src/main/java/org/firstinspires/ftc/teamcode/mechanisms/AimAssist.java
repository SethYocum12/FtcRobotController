package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AimAssist {

    private static final double TURN_GAIN = 0.01;
    private static final double MAX_TURN = 0.15;
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
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

    public double getTagRangeInches(boolean redAlliance) {
        LLResultTypes.FiducialResult tag = vision.getAimTag(redAlliance);

        if (tag == null) {
            return Double.NaN;
        }

        Position position = tag.getTargetPoseCameraSpace()
                .getPosition().toUnit(DistanceUnit.INCH);

        double range = Math.sqrt(
                position.x * position.x
                        + position.y * position.y
                        + position.z * position.z);

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
}
