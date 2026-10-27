package org.firstinspires.ftc.teamcode.pedroPathingAuto;
import com.pedropathing.api.Paths;

import org.firstinspires.ftc.teamcode.dashboard.RobotPanels;
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
import org.firstinspires.ftc.teamcode.mechanisms.servos_and_shooters;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.lazy;
@Autonomous(name = "Red shoot Gar | parkR (1)", group = "Autonomous")
public class RedFrontAutoShoot extends LinearOpMode {

    private Follower follower;
    private final servos_and_shooters mechanisms = new servos_and_shooters();
    private final LIMELIGHT_setup vision = new LIMELIGHT_setup();
    private static final double SHOOTER_TARGET_VELOCITY = 1225.0;
    private static final double INDEXER_POWER = -0.2;
    private static final double TAG_MISSING_SECONDS = 2.0;
    private static final double MAX_FEED_MS = 10000;
    private static final int[] RED_AUDIENCE_TAGS = {34, 35, 36, 37};
    private static final int[] RED_OPPOSITE_TAGS = {30, 31, 32, 33};
    private final ElapsedTime tagMissingTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean hasSeenCell = false;
    private int[] selectedRedTags;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.1184, 8.8286, 270);
    private final Pose path1 = poseFactory.of(56.2596, 18.0853, 270);
    private final Pose point2Start = poseFactory.of(56.2596, 18.0853, 270);
    private final Pose point2 = poseFactory.of(9.3899, 90.6853, 360);
    private final Pose point2Control1 = poseFactory.of(10.3128, 39.7468, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                instant(() -> follower.hold(path1)),
                startShooter(),
                waitForShooter(),
                lazy(() -> feedUntilCellDisappears(selectedRedTags)),
                follow(follower, path2())
        );
    }
    private Command startShooter() {
        return instant(() -> {
            mechanisms.shooterthingy(true, SHOOTER_TARGET_VELOCITY);
        });
    }

    private Command stopShooting() {
        return instant(() -> {
            mechanisms.indexerthingy(false, 0);
            mechanisms.shooterthingy(false, 0);
        });
    }
    private Command waitForShooter() {
        return waitUntil(() -> {
            selectRedCell();

            return selectedRedTags != null
                    && mechanisms.getShooterVelocity() >= SHOOTER_TARGET_VELOCITY * 0.95;
        }).raceWith(waitMs(3000));
    }

    private boolean isCellVisible(int[] tagIds) {
        for (int id : tagIds) {
            if (vision.hasTag(id)) {
                return true;
            }
        }
        return false;
    }


    private boolean cellDisappeared(int[] tagIds) {
        if (!vision.isConnected()
                || vision.getResultAgeMs() < 0
                || vision.getResultAgeMs() > 250
                || vision.getPipelineIndex() != LIMELIGHT_setup.APRILTAG_PIPELINE) {
            tagMissingTimer.reset();
            return false;
        }

        if (isCellVisible(tagIds)) {
            hasSeenCell = true;
            tagMissingTimer.reset();
            return false;
        }



        return hasSeenCell && tagMissingTimer.seconds() >= TAG_MISSING_SECONDS;
    }

    private void selectRedCell() {
        selectedRedTags = null;

        boolean audienceVisible = isCellVisible(RED_AUDIENCE_TAGS);
        boolean oppositeVisible = isCellVisible(RED_OPPOSITE_TAGS);

        if (audienceVisible && !oppositeVisible) {
            selectedRedTags = RED_AUDIENCE_TAGS;
        } else if (oppositeVisible && !audienceVisible) {
            selectedRedTags = RED_OPPOSITE_TAGS;
        }
    }


    private Command feedUntilCellDisappears(int[] tagIds) {
        return conditional(
                () -> tagIds != null
                        && mechanisms.getShooterVelocity() >= SHOOTER_TARGET_VELOCITY * 0.95,
                sequential(
                        instant(() -> {
                            hasSeenCell = true;
                            tagMissingTimer.reset();
                            feedTimer.reset();
                            mechanisms.indexerthingy(true, INDEXER_POWER);
                        }),
                        waitUntil(() -> {
                            double feedSeconds = feedTimer.seconds();
                            mechanisms.feedIndexerWithRecovery(feedSeconds, INDEXER_POWER);
                            return cellDisappeared(tagIds);
                        }).raceWith(waitMs(MAX_FEED_MS)),
                        stopShooting()
                ),
                stopShooting()
        );
    }


    @Override
    public void runOpMode() {
        RobotPanels.attach(this);
        Scheduler.reset();
        mechanisms.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        RobotPanels.preview(path1(), path2());
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
                telemetry.addLine("Path preview only: shooting stage has not been added.");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
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
            mechanisms.indexerthingy(false, 0);
            mechanisms.shooterthingy(false, 0);
            follower.stop();
            vision.limelight_stop();
            RobotPanels.stopped();
        }
    }

    private void addVisionTelemetry() {
        double shooterVelocity = mechanisms.getShooterVelocity();
        RobotPanels.shooter(mechanisms.isShooterEnabled(), shooterVelocity,
                mechanisms.getShooterTargetVelocity());
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
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }
}
