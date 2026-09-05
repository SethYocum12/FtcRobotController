package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MecanumConstants {
    public static String leftFrontMotorName = "front_left";
    public static String leftRearMotorName = "back_left";
    public static String rightFrontMotorName = "front_right";
    public static String rightRearMotorName = "back_right";

    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

    public static double mass = 6.758526; // Estimate in kg
    public static double xVelocity = 80;
    public static double yVelocity = 60;
}
