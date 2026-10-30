package org.firstinspires.ftc.teamcode.pedroPathingAuto;

import org.firstinspires.ftc.teamcode.dashboard.RobotPanels;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;
import org.firstinspires.ftc.teamcode.mechanisms.AutonomousShooting;
import org.firstinspires.ftc.teamcode.mechanisms.servos_and_shooters;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "Red No Gar shoot | Park R mid (1) ", group = "Autonomous")
public class RedNoGarShootParkRMid extends LinearOpMode {

    private Follower follower;
    private final servos_and_shooters mechanisms = new servos_and_shooters();
    private final LIMELIGHT_setup vision = new LIMELIGHT_setup();
    private static final double SHOOTER_TARGET_VELOCITY = 1278.0;
    private static final double INDEXER_POWER = -0.2;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final AutonomousShooting shooting = new AutonomousShooting(
            mechanisms, vision, autoTimer::seconds, true,
            SHOOTER_TARGET_VELOCITY, INDEXER_POWER);

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Front toward the starting wall; rear shooter toward the field.
    private final Pose start = poseFactory.of(62.2681, 133.2729, 90);
    private final Pose path1 = poseFactory.of(56.617, 121.222, 90);
    private final Pose point2Start = poseFactory.of(56.617, 121.222, 90);
    private final Pose point2 = poseFactory.of(41.3493, 23.7627, 0);
    private final Pose point2Control1 = poseFactory.of(60.2783, 107.2883, 0);
    private final Pose point2Control2 = poseFactory.of(55.5511, 94.4529, 0);
    private final Pose point2Control3 = poseFactory.of(96.1952, 56.0919, 0);
    private final Pose point3Start = poseFactory.of(41.3493, 23.7627, 0);
    private final Pose point3 = poseFactory.of(14.0105, 92.5297, 0);
    private final Pose point3Control1 = poseFactory.of(15.2437, 68.1668, 0);

    // All shooting autos share the tag trigger and the 20-second departure rule.
    public Command autoRoutine() {
        return sequential(
                shooting.approachAndShoot(follower, path1(), path1),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        RobotPanels.attach(this);
        Scheduler.reset();
        mechanisms.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        RobotPanels.preview(path1(), path2(), path3());
        follower.update();
        RobotPanels.follower(follower);

        try {
            vision.init(hardwareMap);
            vision.limelight_start();
            // Preview tags during INIT without scheduling any movement.
            while (opModeInInit()) {
                vision.main();
                RobotPanels.follower(follower);
                addVisionTelemetry();
                telemetry.addLine("Tag-triggered shooting; next path starts at 20 seconds after START.");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            autoTimer.reset();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                vision.main();
                follower.update();
                RobotPanels.follower(follower);
                Scheduler.execute();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());
                addVisionTelemetry();

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
                idle();
            }
        } finally {
            Scheduler.reset();
            shooting.stop();
            follower.stop();
            vision.limelight_stop();
            RobotPanels.stopped();
        }
    }

    private void addVisionTelemetry() {
        double shooterVelocity = mechanisms.getShooterVelocity();
        RobotPanels.shooter(mechanisms.isShooterEnabled(), shooterVelocity,
                mechanisms.getShooterTargetVelocity());
        telemetry.addData("Shooting", shooting.getStatus());
        telemetry.addData("Selected cell", shooting.getSelectedCell());
        telemetry.addData("Auto seconds", "%.2f", autoTimer.seconds());
        telemetry.addData("Depart at (seconds)", AutonomousShooting.DEPART_AT_SECONDS);
        telemetry.addData("Feed seconds", "%.2f", shooting.getFeedSeconds());
        telemetry.addData("Shooter velocity (ticks/s)", shooterVelocity);
        telemetry.addData("Selected cell visible", shooting.selectedCellVisible());
        telemetry.addData("Vision", vision.getStatus());
        telemetry.addData("Limelight connected", vision.isConnected());
        telemetry.addData("Vision requested pipeline", LIMELIGHT_setup.APRILTAG_PIPELINE);
        telemetry.addData("Vision pipeline", vision.getPipelineIndex());
        telemetry.addData("Vision pipeline type", vision.getPipelineType());
        telemetry.addData("Vision data age (ms)", vision.getResultAgeMs());
        telemetry.addData("Tag visible", vision.tagIdTele());
        telemetry.addData("Tag IDs", vision.getTagIds());
        if (vision.tagIdTele()) {
            telemetry.addData("Primary target TX / TY (degrees)", "%.2f / %.2f",
                    vision.returnLLResultTx(), vision.returnLLResultTy());
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2Control3, point2)
                .linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }
}
