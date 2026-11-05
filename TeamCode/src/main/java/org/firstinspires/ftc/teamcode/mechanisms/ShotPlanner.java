package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.math.Velocity;

public class ShotPlanner {

    private final AimAssist aim;
    private final servos_and_shooters shooter;
    private static final double MAX_STILL_SPEED = 1.0;
    private static final double MAX_STILL_TURN_RATE = Math.toRadians(5);
    private static final double STILL_TIME_SECONDS = 2.0;
    private final ElapsedTime stillTimer = new ElapsedTime();
    private boolean wasStationary = false;
    private boolean settled;

    public ShotPlanner(AimAssist aim, servos_and_shooters shooter) {

        this.aim = aim;
        this.shooter = shooter;
    }
    public void reset() {
        wasStationary = false;
        settled = false;
        stillTimer.reset();
    }

    private boolean stationaryShotReady(Velocity velocity) {
        boolean stationary = velocity != null &&
                Math.hypot(velocity.vx,velocity.vy) <= MAX_STILL_SPEED
                && Math.abs(velocity.omega) <= MAX_STILL_TURN_RATE;


        if (!stationary) {
            wasStationary = false;
            stillTimer.reset();
            return false;
        }

        if (!wasStationary) {
            stillTimer.reset();
            wasStationary = true;
        }

        return stillTimer.seconds() >= STILL_TIME_SECONDS;

        }




    public ShotSolution evaluate(boolean redAlliance, Velocity velocity) {
        settled = stationaryShotReady(velocity);
        double turn = aim.getTurn(redAlliance);
        double target = shooter.getShooterTargetVelocity();
        String status;
        boolean ready = false;

        if (!aim.hasAimTarget(redAlliance)) {
            status = "No fresh alliance tag";
        } else if (!shooter.isShooterEnabled()) {
            status = "Shooter off";
        } else if (!aim.isAligned(redAlliance)) {
            status = "Aiming";
        } else if (!shooter.isShooterReady()) {
            status = "Waiting for 97% flywheel speed";
        } else if (!settled) {
            status = "Waiting to settle for 2 seconds";
        } else {
            String problem = calibrationProblem(redAlliance);
            if (problem != null) {
                status = problem;
            } else {
                double expected = ShotCalibration.velocityFor(aim.getTagRangeInches(redAlliance));
                ready = Math.abs(expected - target) <= 0.5;
                status = ready ? "Stationary shot ready" : "Updating flywheel target";
            }
        }

        return new ShotSolution(turn, target, ready, status);
    }

    private String calibrationProblem(boolean redAlliance) {
        if (!ShotCalibration.ENABLE_STATIONARY_SMART_SHOTS) return "Calibration not enabled";
        if (!ShotCalibration.RANGE_VERIFIED) return "Verify camera range with tape";
        if (!ShotCalibration.AIM_VERIFIED) return "Verify shooter aim offset";
        if (aim.getAimTagId(redAlliance) != ShotCalibration.CALIBRATED_TAG_ID)
            return "Selected tag has no shot calibration";
        if (!ShotCalibration.isValidTable()) return "Need valid measured shot table";
        double target = ShotCalibration.velocityFor(aim.getTagRangeInches(redAlliance));
        if (!Double.isFinite(target)) return "Outside measured camera-tag range";
        if (target > shooter.getMaxShooterVelocity()) return "Target exceeds configured motor speed";
        return null;
    }

    public double requestedVelocity(boolean redAlliance, double manualVelocity, boolean practice) {
        if (practice || !aim.hasAimTarget(redAlliance) || calibrationProblem(redAlliance) != null)
            return manualVelocity;
        return ShotCalibration.velocityFor(aim.getTagRangeInches(redAlliance));
    }

    /** Practice permission measures stationary shots before calibration exists. */
    public boolean practiceFeedReady(boolean redAlliance) {
        return settled && aim.hasAimTarget(redAlliance)
                && aim.isAligned(redAlliance) && shooter.isShooterReady();
    }

}
