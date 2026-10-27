package org.firstinspires.ftc.teamcode.mechanisms;

public class AimAssist {

    private static final double TURN_GAIN = 0.01;
    private static final double MAX_TURN = 0.15;
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
    private final LIMELIGHT_setup vision;
    public AimAssist(LIMELIGHT_setup vision){
        this.vision = vision;
    }

    public double calculateTurn(double errorDegrees) {
        if (Math.abs(errorDegrees) <= AIM_TOLERANCE_DEGREES) {
            return 0;
        }

        double turn = errorDegrees * TURN_GAIN;
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));
    }

    public boolean hasAimTarget(boolean redAlliance) {
        if (!vision.tyTele()) {
            return false;
        }

        double tagId = vision.returnTagId();

        if (redAlliance) {
            return tagId >= 30 && tagId <= 37;
        }

        return tagId >= 38 && tagId <= 45;
    }
    public double getTurn(boolean redAlliance) {
        if (!hasAimTarget(redAlliance)) {
            return 0;
        }

        return calculateTurn(vision.returnLLResultTy());
    }

}
