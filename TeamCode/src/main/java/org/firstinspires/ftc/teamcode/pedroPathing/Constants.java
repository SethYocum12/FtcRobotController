package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.x configuration for Mimir.
 *
 * A Follower needs exactly three things:
 *   1. a Localizer  - tells it WHERE the robot is        (our Pinpoint)
 *   2. a Drivetrain - lets it MOVE the robot             (our mecanum wheels)
 *   3. an Algorithm - decides HOW HARD to drive the motors to reach the path (Foresight)
 *
 * In 3.x each of those is configured with a small lambda that sets ConfigVar fields,
 * instead of the .method().method() builder chain that 2.x used.
 */
public class Constants {

    /** 1. DRIVETRAIN - names must match the Robot Configuration on the Driver Station exactly. */
    public static final MecanumConfig DRIVE = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.backLeftName.set("back_left");
        c.frontRightName.set("front_right");
        c.backRightName.set("back_right");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    /** 2. LOCALIZER - the goBILDA Pinpoint odometry computer. */
    public static final PinpointConfig LOCALIZER = new PinpointConfig(c -> {
        c.name.set("odometry");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.offsetUnits.set(DistanceUnit.INCH);

        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // TODO(human): set the two pod offsets.
        //
        // The Pinpoint needs to know where each odometry pod sits relative to the
        // robot's center of rotation, so it can separate "the robot drove forward"
        // from "the robot spun and dragged the pod sideways".
        //
        // Pedro's coordinate frame, looking down at the robot from above:
        //     +X = forward (the direction the robot drives)
        //     +Y = left
        //
        //   c.xPodOffset.set( ... );   // the FORWARD pod's sideways (Y) distance from center
        //   c.yPodOffset.set( ... );   // the STRAFE pod's forward (X) distance from center
        //
        // Your old 2.x file had forwardPodY(-5) and strafePodX(0.5), so those are a
        // reasonable starting guess - but they were never verified on the real robot.
    });

    /**
     * 3. ALGORITHM - Foresight looks ahead on the path and decides motor powers.
     *
     * These numbers came over from your 2.x constants. The velocity/deceleration values
     * are real measurements; the PID values are still untuned starting points.
     * Note: 3.x has no "mass" setting - the deceleration numbers replace it.
     */
    public static final ForesightConfig ALGORITHM = new ForesightConfig(c -> {
        // How fast the robot can actually go, in inches/sec (measured).
        c.maxAchievableForwardVelocity.set(67.70526843934547);
        c.maxAchievableStrafeVelocity.set(55.14854515255905);

        // How fast it coasts to a stop with zero power, in inches/sec^2 (measured, negative).
        c.naturalForwardDeceleration.set(-25.765493295492043);
        c.naturalStrafeDeceleration.set(-22.01971810239012);

        // Feedback controllers: Controller.pid(P, I, D). Tune these with the
        // Pedro tuning OpModes (com.pedropathing:tuning) before trusting an auto run.
        c.forwardTranslational.set(Controller.pid(0.025, 0.0, 0.003));
        c.strafeTranslational.set(Controller.pid(0.025, 0.0, 0.003));
        c.headingFeedback.set(Controller.pid(0.4, 0.0, 0.002));
    });

    /** Build a ready-to-use Follower. Every OpMode calls this. */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, LOCALIZER),
                new Mecanum(hardwareMap, DRIVE),
                new Foresight(ALGORITHM)
        );
    }
}
