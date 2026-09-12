package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.control.PIDFCoefficients;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FollowerConstants {
    public static double mass = MecanumConstants.mass;
    public static double xVelocity = MecanumConstants.xVelocity;
    public static double yVelocity = MecanumConstants.yVelocity;

    public static PIDFCoefficients translationalPIDFCoefficients = new PIDFCoefficients(0.1, 0, 0, 0);
    public static PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(2, 0, 0, 0);
    public static PIDFCoefficients drivePIDFCoefficients = new PIDFCoefficients(0.01, 0, 0, 0);
}
