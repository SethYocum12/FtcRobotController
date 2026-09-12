package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LConstants {
    // These are placeholders. You'll need to update these based on your robot's sensors!
    public static double forwardTicksToInches = 0.001989436789;
    public static double strafeTicksToInches = 0.001989436789;
    public static double turnTicksToInches = 0.001989436789;

    public static String leftEncoder_HardwareMapName = "front_left";
    public static String rightEncoder_HardwareMapName = "back_right";
    public static String strafeEncoder_HardwareMapName = "front_right";

    public static double leftEncoderDirection = Encoder.REVERSE;
    public static double rightEncoderDirection = Encoder.REVERSE;
    public static double strafeEncoderDirection = Encoder.FORWARD;

    public static Pose leftEncoderPos = new Pose(0, 6, 0);
    public static Pose rightEncoderPos = new Pose(0, -6, 0);
    public static Pose strafeEncoderPos = new Pose(-6, 0, Math.toRadians(90));
}
