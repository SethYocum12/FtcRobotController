package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(-5)
                    .strafePodX(0.5)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName("odometry")
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                    )
                    .forwardEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.REVERSED
                    )
                    .strafeEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.REVERSED
                    );
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.5)
            .forwardZeroPowerAcceleration(-25.765493295492043)
            .lateralZeroPowerAcceleration(-22.01971810239012)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.025, 0, 0.003, 0.0245))
            .headingPIDFCoefficients(new PIDFCoefficients(0.4,0,0.002,0.0245));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left ")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(67.70526843934547)
            .yVelocity(55.14854515255905);


    public static PathConstraints pathConstraints =
            new PathConstraints(0.99,
                    100, 1,
                    1
            );public static Follower createFollower(HardwareMap hardwareMap) {


        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
