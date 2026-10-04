package org.firstinspires.ftc.teamcode.learning;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.api.Paths.line;

/** This SAME file is compiled by both desktop and Android builds. */
public abstract class LearningAuto extends OpMode {
    // Student exercise: predict the endpoint before changing this distance.
    public static final double DISTANCE_INCHES = 12.0;
    private static final double TIMEOUT_SECONDS = 12.0;
    protected Follower follower;
    private Pose start;
    private Pose target;
    private double startedAt;
    private String result = "Ready";
    private boolean finished;

    protected abstract String movement();

    @Override public void init() {
        follower = LearningRobot.create(hardwareMap);
        PoseFactory degrees = PoseFactory.degrees();
        start = degrees.of(72, 72, 90);
        if (movement().equals("left")) {
            target = degrees.of(72 - DISTANCE_INCHES, 72, 90);
        } else if (movement().equals("forward")) {
            target = degrees.of(72, 72 + DISTANCE_INCHES, 90);
        } else {
            target = degrees.of(72, 72, 180);
        }
        follower.setPose(start);
        telemetry.addData("Environment", LearningRobot.environment());
        telemetry.addData("Task", movement());
        telemetry.addLine("Start heading 90 degrees. Left decreases field X; forward increases field Y.");
    }

    @Override public void start() {
        follower.setPose(start);
        startedAt = getRuntime();
        result = "Running";
        finished = false;
        if (movement().equals("turn")) follower.hold(target);
        else follower.follow(line(start, target).linear(start, target));
    }

    @Override public void loop() {
        follower.update();
        Pose pose = follower.pose();
        double distanceError = pose.distance(target);
        double headingError = Math.atan2(Math.sin(target.heading() - pose.heading()),
                Math.cos(target.heading() - pose.heading()));
        if (!finished) {
            if (distanceError < 0.75 && Math.abs(headingError) < Math.toRadians(3)
                    && getRuntime() - startedAt > 0.3) {
                result = "Reached target";
                finished = true;
                follower.stop();
            } else if (getRuntime() - startedAt > TIMEOUT_SECONDS) {
                result = "Timed out - inspect pose and configuration";
                finished = true;
                follower.stop();
            }
        }
        telemetry.addData("Environment", LearningRobot.environment());
        telemetry.addData("Result", result);
        telemetry.addData("X / Y (inches)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Target X / Y", "%.2f / %.2f", target.x(), target.y());
        telemetry.addData("Distance error (inches)", "%.2f", distanceError);
        telemetry.update();
    }

    @Override public void stop() { if (follower != null) follower.stop(); }
}
