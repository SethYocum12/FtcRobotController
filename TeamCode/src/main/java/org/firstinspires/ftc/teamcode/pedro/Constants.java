package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

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

    public static PinpointConfig LOCALIZER = new PinpointConfig(c -> {
        c.name.set("odometry");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(7.057734812338521);
        c.yPodOffset.set(-3.002521484855592);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    // Foresight AutoTune results supplied on October 15, 2026.
    public static final ForesightConfig ALGORITHM = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.2557425841680563);
        Controller secondaryTranslationalForward = Controller.proportional(0.09449005128239221);
        Controller primaryTranslationalLateral = Controller.proportional(0.7409847171804026);
        Controller secondaryTranslationalLateral = Controller.proportional(0.2737740535218635);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
        c.coast.set(Controller.proportionalFeedforward(0.017471546002682734));
        c.brake.set(Controller.proportionalFeedforward(0.014850814102280323));
        c.headingFeedback.set(Controller.proportional(4.810906986140562));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05905173952327245, 0.005423407156124606));
        c.linearBrakeCoefficients.set(Matrix.diag(0.05602441891661327, 0.03505442328251957));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025863773802547825, 0.0022539721989514594));
        c.maxAchievableForwardVelocity.set(57.68177972711395);
        c.maxAchievableStrafeVelocity.set(49.923464660018254);
        c.naturalForwardDeceleration.set(31.626293589471306);
        c.naturalStrafeDeceleration.set(59.27211441835567);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, LOCALIZER),
                new Mecanum(hardwareMap, DRIVE),
                new Foresight(ALGORITHM)
        );
    }
}
