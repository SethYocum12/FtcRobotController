package org.firstinspires.ftc.teamcode.mechanisms;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.dashboard.RobotPanels;
import com.pedropathing.math.Pose;


public class PositionHold {
    private Follower follower;
    private boolean holding = false;

    public void init(HardwareMap hardwareMap) {
        double originalThreshold = Constants.DRIVE.powerThreshold.get();
        Constants.DRIVE.powerThreshold.set(0.0);

        try {
            follower = Constants.create(hardwareMap);
        } finally {
            Constants.DRIVE.powerThreshold.set(originalThreshold);
        }
        follower.setPose(new Pose(RobotPanels.TELEOP_START_X, RobotPanels.TELEOP_START_Y,
                Math.toRadians(RobotPanels.TELEOP_START_HEADING_DEGREES)));
    }

    public void startHolding() {
        if (!holding) {
            follower.localizer.update();
            follower.hold(follower.pose());
            holding = true;
        }
    }
    public void stopHolding() {
        if (holding) {
            follower.stop();
            holding = false;
        }
    }
    public void update() {
        if (holding) {
            follower.update();
        }
    }

    /** Updates tracking only; never calls the follower's motor-control update. */
    public void updateTracking() {
        follower.localizer.update();
    }

    public Pose getPose() {
        return follower.pose();
    }
}
