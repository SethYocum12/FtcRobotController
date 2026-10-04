package org.firstinspires.ftc.teamcode.learning;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
/** Android-only configuration using a COPY of the team's current robot constants. */
public final class LearningRobot {
    public static Follower create(HardwareMap hardwareMap) {
        return org.firstinspires.ftc.teamcode.pedroPathing.Constants.create(hardwareMap);
    }
    public static String environment() { return "ROBOT - verify physical localization and tuning first"; }
}
