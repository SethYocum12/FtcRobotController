package org.firstinspires.ftc.teamcode.learning;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
/** Desktop-only configuration. Never copy simulator tuning values onto the robot. */
public final class LearningRobot {
    public static Follower create(HardwareMap hardwareMap) {
        return org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants.create(hardwareMap);
    }
    public static String environment() { return "SIMULATOR - MecDynamic"; }
}
