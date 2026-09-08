package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower createFollower(HardwareMap hardwareMap) {
        // Build the Follower using our local constants
        MecanumConstants mecanumConstants = new MecanumConstants()
                .leftFrontMotorName(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.leftFrontMotorName)
                .leftRearMotorName(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.leftRearMotorName)
                .rightFrontMotorName(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.rightFrontMotorName)
                .rightRearMotorName(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.rightRearMotorName)
                .leftFrontMotorDirection(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.leftFrontMotorDirection)
                .leftRearMotorDirection(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.leftRearMotorDirection)
                .rightFrontMotorDirection(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.rightFrontMotorDirection)
                .rightRearMotorDirection(org.firstinspires.ftc.teamcode.pedroPathing.constants.MecanumConstants.rightRearMotorDirection);

        ThreeWheelConstants threeWheelConstants = new ThreeWheelConstants()
                .leftEncoder_HardwareMapName(LConstants.leftEncoder_HardwareMapName)
                .rightEncoder_HardwareMapName(LConstants.rightEncoder_HardwareMapName)
                .strafeEncoder_HardwareMapName(LConstants.strafeEncoder_HardwareMapName)
                .leftEncoderDirection(LConstants.leftEncoderDirection)
                .rightEncoderDirection(LConstants.rightEncoderDirection)
                .strafeEncoderDirection(LConstants.strafeEncoderDirection);

        com.pedropathing.follower.FollowerConstants followerConstants = new com.pedropathing.follower.FollowerConstants()
                .mass(FollowerConstants.mass)
                .translationalPIDFCoefficients(FollowerConstants.translationalPIDFCoefficients)
                .headingPIDFCoefficients(FollowerConstants.headingPIDFCoefficients);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelLocalizer(threeWheelConstants)
                .build();
    }
}
