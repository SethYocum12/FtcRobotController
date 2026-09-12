package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.panels.Panels;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;
import org.firstinspires.ftc.teamcode.mechanisms.servos;

@TeleOp(name = "Pedro PIDF Tuner", group = "Mimir")
public class Pedro_Pathing extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // Mimir Mechanisms
    private final IMU_setup the_imu = new IMU_setup();
    private final servos servos = new servos();
    private final LIMELIGHT_setup the_limelight = new LIMELIGHT_setup();

    // Movement and Toggle Variables (Matching Mimir_2026_Mec)
    private boolean intakeToggle = false;
    private boolean intakeTrigger = false;
    private boolean lastIntakeTrigger = false;
    private final double intakeSpeed = -1;

    private boolean turboToggle = false;
    private boolean turboTrigger = false;
    private boolean lastTurboTrigger = false;

    @Override
    public void runOpMode() {
        // Initialize Panels (The Dashboard)
        Panels.start(hardwareMap.appContext);

        // Initialize Pedro Pathing Follower using our new Constants file
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize Mimir Mechanisms
        servos.init(hardwareMap);
        the_imu.init(hardwareMap);
        the_limelight.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        follower.startTeleopDrive();
        the_limelight.limelight_start();

        while (opModeIsActive()) {
            follower.update();

            /* Pedro Pathing Drive Control */
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y, // Forward/Backward
                    -gamepad1.left_stick_x, // Strafe
                    -gamepad1.right_stick_x, // Rotation
                    true                    // Field Centric
            );

            /* Mimir Specific Logic (Servos/Limelight/Intake) */
            intakeTrigger = gamepad2.a;
            turboTrigger = gamepad1.right_bumper;

            // Turbo Toggle
            if (turboTrigger && !lastTurboTrigger) {
                turboToggle = !turboToggle;
            }
            lastTurboTrigger = turboTrigger;

            // Intake Toggle
            if (intakeTrigger && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;
            }
            lastIntakeTrigger = intakeTrigger;

            // Apply Toggle States
            servos.servos_move(intakeToggle, intakeSpeed);
            the_limelight.main();

            // Telemetry
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("IMU Heading", Math.toDegrees(the_imu.getHeading()));
            telemetry.addData("Turbo Mode?", turboToggle);
            telemetry.addData("Intake Mode?", intakeToggle);
            telemetry.update();
        }
    }
}
