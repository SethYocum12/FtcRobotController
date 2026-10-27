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
import org.firstinspires.ftc.teamcode.mechanisms.servos_and_shooters;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.lazy;
@Autonomous(name = "Blue shoot Gar | parkR (1)", group = "Autonomous")
public class BlueFrontAutoShoot extends LinearOpMode {

    private Follower follower;
    private final servos_and_shooters mechanisms = new servos_and_shooters();
    private final LIMELIGHT_setup vision = new LIMELIGHT_setup();
    private static final double SHOOTER_TARGET_VELOCITY = 1278.0;
    private static final double INDEXER_POWER = -0.2;
    private static final int[] BLUE_AUDIENCE_TAGS = {38, 39, 40, 41};
    private static final int[] BLUE_OPPOSITE_TAGS = {42, 43, 44, 45};
    private static final double TAG_MISSING_SECONDS = 2.0;
    private static final double MAX_FEED_MS = 10000;
    private final ElapsedTime tagMissingTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean hasSeenCell = false;
    private int[] selectedBlueTags;
    private String selectedCell = "None";
    private String shootingStatus = "Waiting for START";
    private boolean stoppedForTagLoss = false;
    private double lastFeedSeconds = 0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(83.7807, 133.4028, 90);
    private final Pose path1 = poseFactory.of(84.0404, 131.5349, 90);
    private final Pose point2Start = poseFactory.of(84.0404, 124.5349, 90);
    private final Pose point2 = poseFactory.of(131.7154, 51.2796, 180);
    private final Pose point2Control1 = poseFactory.of(121.7803, 106.3743, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                instant(() -> follower.hold(path1)),
                startShooter(),
                waitForShooter(),
                lazy(() -> feedUntilCellDisappears(selectedBlueTags)),
                follow(follower, path2())
        );
    }
    private Command startShooter() {
        return instant(() -> {
            shootingStatus = "Waiting for shooter speed and one blue cell";
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
            selectBlueCell();

            return selectedBlueTags != null
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

    private void selectBlueCell() {
        selectedBlueTags = null;
        selectedCell = "None";

        boolean audienceVisible = isCellVisible(BLUE_AUDIENCE_TAGS);
        boolean oppositeVisible = isCellVisible(BLUE_OPPOSITE_TAGS);

        if (audienceVisible && !oppositeVisible) {
            selectedBlueTags = BLUE_AUDIENCE_TAGS;
            selectedCell = "Blue audience (38-41)";
        } else if (oppositeVisible && !audienceVisible) {
            selectedBlueTags = BLUE_OPPOSITE_TAGS;
            selectedCell = "Blue opposite (42-45)";
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
                            stoppedForTagLoss = false;
                            shootingStatus = "Feeding";
                            mechanisms.indexerthingy(true, INDEXER_POWER);
                        }),
                        waitUntil(() -> {
                            mechanisms.feedIndexerWithRecovery(feedTimer.seconds(), INDEXER_POWER);
                            stoppedForTagLoss = cellDisappeared(tagIds);
                            return stoppedForTagLoss;
                        }).raceWith(waitMs(MAX_FEED_MS)),
                        instant(() -> {
                            lastFeedSeconds = feedTimer.seconds();
                            shootingStatus = stoppedForTagLoss
                                    ? "Stopped: selected tags missing for 2 seconds"
                                    : "Stopped: 7-second feeding limit";
                        }),
                        stopShooting()
                ),
                sequential(
                        instant(() -> shootingStatus = tagIds == null
                                ? "Skipped: no unambiguous blue cell detected"
                                : "Skipped: shooter below target speed"),
                        stopShooting()
                )
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
                telemetry.addLine("Blue shoot and park: START runs the path and shooter.");
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
        telemetry.addData("Shooting", shootingStatus);
        telemetry.addData("Selected cell", selectedCell);
        telemetry.addData("Feed seconds", "%.2f",
                "Feeding".equals(shootingStatus) ? feedTimer.seconds() : lastFeedSeconds);
        telemetry.addData("Shooter velocity (ticks/s)", shooterVelocity);
        telemetry.addData("Selected cell visible",
                selectedBlueTags != null && isCellVisible(selectedBlueTags));
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
