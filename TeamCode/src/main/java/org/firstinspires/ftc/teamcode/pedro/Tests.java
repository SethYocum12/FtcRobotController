package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.tuning.autotune.DisplayName;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.function.Function;
import java.util.function.Supplier;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

public class Tests extends Procedure {
    enum Test {
        @DisplayName("Hold Test")
        HOLD,
        @DisplayName("Line Test")
        LINE,
        @DisplayName("Single Forward 24 Inches")
        SINGLE_FORWARD,
        @DisplayName("Single Left 24 Inches")
        SINGLE_LEFT,
        @DisplayName("Single Turn 90 Degrees")
        SINGLE_TURN,
        @DisplayName("Single Curve 24 by 24 Inches")
        SINGLE_CURVE,
        @DisplayName("Curve Test")
        CURVED,
        @DisplayName("Interpolation Test")
        INTERPOLATION_CURVED,
        @DisplayName("Localization Test")
        LOCALIZATION,
        @DisplayName("Odometry Test")
        ODOMETRY,
        @DisplayName("Driving Test")
        DRIVING,
        @DisplayName("Pose Test")
        POSE
    }
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    Function<HardwareMap, Localizer> localizerFunction;
    Supplier<Algorithm> algorithmSupplier;
    Function<HardwareMap, Follower> followerFunction;

    public Tests(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction, Supplier<Algorithm> algorithmSupplier) {
        super("Tests", "A procedure for testing the Follower.");
        this.drivetrainFunction = drivetrainFunction;
        this.localizerFunction = localizerFunction;
        this.algorithmSupplier = algorithmSupplier;
    }

    @Override
    public void run() throws InterruptedException {
        boolean completed = false;
        boolean algorithm = true, localizer = true, drivetrain = true;

        if (algorithmSupplier == null)
            algorithm = false;

        if (localizerFunction == null)
            localizer = false;

        if (drivetrainFunction == null)
            drivetrain = false;

        if (algorithm && localizer && drivetrain)
            followerFunction = (hardwareMap) -> new Follower(localizerFunction.apply(hardwareMap), drivetrainFunction.apply(hardwareMap), algorithmSupplier.get());

        Inputs inputs = inputs("Select", "Select");
        Inputs.Field<Test> selectedTest = inputs.e("Test", Test.class).withDefault(Test.LINE);
        Inputs.Field<Double> distance = inputs.d("Distance").withDefault(48.0);

        awaitInputs(inputs);

        switch (selectedTest.get()) {
            case HOLD:
                if (!algorithm)
                    throw new IllegalArgumentException("Algorithm is required for Hold Test.");
                completed = runOpMode(new TestsHold(followerFunction));
                break;
            case LINE:
                if (!algorithm)
                    throw new IllegalArgumentException("Algorithm is required for Hold Test.");
                completed = runOpMode(new TestsLine(followerFunction, distance.get()));
                break;
            case SINGLE_FORWARD:
                if (followerFunction == null)
                    throw new IllegalArgumentException("Drivetrain, localizer and algorithm are required.");
                completed = runOpMode(new TestsSingleMove(followerFunction, false));
                break;
            case SINGLE_LEFT:
                if (followerFunction == null)
                    throw new IllegalArgumentException("Drivetrain, localizer and algorithm are required.");
                completed = runOpMode(new TestsSingleMove(followerFunction, true));
                break;
            case SINGLE_TURN:
                if (followerFunction == null)
                    throw new IllegalArgumentException("Drivetrain, localizer and algorithm are required.");
                completed = runOpMode(new TestsSingleTurn(followerFunction));
                break;
            case SINGLE_CURVE:
                if (followerFunction == null)
                    throw new IllegalArgumentException("Drivetrain, localizer and algorithm are required.");
                completed = runOpMode(new TestsSingleCurve(followerFunction));
                break;
            case CURVED:
                if (!algorithm)
                    throw new IllegalArgumentException("Algorithm is required for Hold Test.");
                completed = runOpMode(new TestsCurve(followerFunction, distance.get()));
                break;
            case INTERPOLATION_CURVED:
                if (!algorithm)
                    throw new IllegalArgumentException("Algorithm is required for Hold Test.");
                completed = runOpMode(new TestsInterpolation(followerFunction, distance.get()));
                break;
            case LOCALIZATION:
                if (!drivetrain)
                    throw new IllegalArgumentException("Drivetrain is required for Localization Test.");
                if (!localizer)
                    throw new IllegalArgumentException("Localizer is required for Localization Test.");
                completed = runOpMode(new TestsLocalization(drivetrainFunction, localizerFunction));
                break;
            case ODOMETRY:
                if (!drivetrain)
                    throw new IllegalArgumentException("Drivetrain is required for Odometry Test.");
                if (!localizer)
                    throw new IllegalArgumentException("Localizer is required for Odometry Test.");
                completed = runOpMode(new TestsOdometry(drivetrainFunction, localizerFunction));
                if (!completed)
                    abort("Failed odometry test. Please check your odometry pods and ensure they are functioning correctly.");
                break;
            case POSE:
                if (!localizer)
                    throw new IllegalArgumentException("Localizer is required for Pose Test.");
                completed = runOpMode(new TestsPose(localizerFunction));
                break;
            case DRIVING:
                if (!drivetrain)
                    throw new IllegalArgumentException("Drivetrain is required for Driving Test.");
                completed = runOpMode(new TestsDriving(drivetrainFunction));
                break;
        }

        result("Completed", completed);
    }
}

class TestsHold extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;

    public TestsHold(Function<HardwareMap, Follower> followerFunction) {
        super("Hold Test", "Tests the Follower's ability to hold a position.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());
        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.hold(Pose.zero());
        while (opModeIsActive()) {
            follower.update();
        }
        return true;
    }
}

class TestsLine extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;
    double distance;

    public TestsLine(Function<HardwareMap, Follower> followerFunction, double distance) {
        super("Line Test", "Tests the Follower's ability to follow a line.", true);
        this.followerFunction = followerFunction;
        this.distance = distance;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = line(Pose.zero(), new Pose(distance,0, 0)).constant(0);
        Path path2 = line(new Pose(distance,0, 0), Pose.zero()).constant(0);

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        return true;
    }
}

class TestsCurve extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;
    double distance;

    public TestsCurve(Function<HardwareMap, Follower> followerFunction, double distance) {
        super("Curve Test", "Tests the Follower's ability to follow a curve.", true);
        this.followerFunction = followerFunction;
        this.distance = distance;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(distance + 0,0), new Pose(distance,distance)).tangent();
        Path path2 = curve(new Pose(distance,distance), new Pose(distance,0), Pose.zero()).tangent();

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        return true;
    }
}

class TestsInterpolation extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;
    double distance;

    public TestsInterpolation(Function<HardwareMap, Follower> followerFunction, double distance) {
        super("Interpolation Curve Test", "Tests the Follower's ability to follow a curve with several interpolations.", true);
        this.followerFunction = followerFunction;
        this.distance = distance;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(distance + 0,0), new Pose(distance,distance)).heading((curve, t) -> Math.PI);
        Path path2 = curve(new Pose(distance,distance), new Pose(distance,0), Pose.zero()).heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1.0, Interpolator.constant(0)));

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        return true;
    }
}

class TestsLocalization extends TuningOpMode<Boolean> {
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    Function<HardwareMap, Localizer> localizerFunction;

    public TestsLocalization(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction) {
        super("Localization Test", "Verifies localization and manual control.", true);
        this.drivetrainFunction = drivetrainFunction;
        this.localizerFunction = localizerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());

        while (opModeIsActive()) {
            drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x), true);
            localizer.update();
            telemetry.addData("Pose", localizer.pose());
            telemetry.update();
        }
        return true;
    }
}

class TestsOdometry extends TuningOpMode<Boolean> {
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    Function<HardwareMap, Localizer> localizerFunction;

    public enum Test {
        FORWARD,
        LEFT,
        TURN,
        IDLE
    }

    private Test test = Test.FORWARD;
    public static double POWER = 0.5;

    double totalHeading;
    double prevHeading;

    private final ElapsedTime timer = new ElapsedTime();

    private boolean passedX = false;
    private boolean passedY = false;
    private boolean passedHeading = false;

    public TestsOdometry(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction) {
        super("Localization Test", "Verifies localization and manual control.", true);
        this.drivetrainFunction = drivetrainFunction;
        this.localizerFunction = localizerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());

        timer.reset();

        while (opModeIsActive()) {
            localizer.update();

            if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y() || localizer.pose().heading() != Pose.zero().heading()) {
                double currentHeading = localizer.pose().heading();
                totalHeading += Angle.normalizeSigned(currentHeading - prevHeading);
                prevHeading = currentHeading;
            }

            switch (test) {
                case FORWARD:
                    drivetrain.drive(new DrivePowers(POWER, 0, 0), false);

                    if (timer.seconds() > 1) {
                        test = Test.LEFT;
                        timer.reset();
                    }
                    break;
                case LEFT:
                    drivetrain.drive(new DrivePowers(0, POWER, 0), false);

                    if (timer.seconds() > 1) {
                        test = Test.TURN;
                        timer.reset();

                        prevHeading = localizer.pose().heading();
                        totalHeading = 0;
                    }
                    break;
                case TURN:
                    drivetrain.drive(new DrivePowers(0, 0, POWER), false);

                    if (timer.seconds() > 1) {
                        test = Test.IDLE;
                        timer.reset();
                    }
                    break;
                case IDLE:
                    drivetrain.drive(new DrivePowers(0, 0, 0), true);

                    telemetry.addData("Test", "Completed");

                    Pose pose = localizer.pose();

                    if (pose.x() < 0)
                        telemetry.addData("xPod Direction", "Flipped");
                    else if (pose.x() < 2)
                        telemetry.addData("xPod Resolution", "Too high");
                    else if (pose.x() > 144)
                        telemetry.addData("xPod Resolution", "Too low");
                    else {
                        telemetry.addData("xPod", "Good");
                        passedX = true;
                    }

                    if (pose.y() < 0)
                        telemetry.addData("yPod Direction", "Flipped");
                    else if (pose.y() < 2)
                        telemetry.addData("yPod Resolution", "Too high");
                    else if (pose.y() > 144)
                        telemetry.addData("yPod Resolution", "Too low");
                    else {
                        telemetry.addData("yPod", "Good");
                        passedY = true;
                    }

                    if (totalHeading < 0)
                        telemetry.addData("Heading Direction", "Flipped");
                    else if (totalHeading < 0.02)
                        telemetry.addData("Heading Resolution", "Too high");
                    else if (totalHeading > 2 * Math.PI)
                        telemetry.addData("Heading Resolution", "Too low");
                    else {
                        telemetry.addData("Heading", "Good");
                        passedHeading = true;
                    }

                    break;
            }

            telemetry.addData("Pose", localizer.pose());
            telemetry.update();
        }
        return passedX && passedY && passedHeading;
    }
}

class TestsDriving extends TuningOpMode<Boolean> {
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public TestsDriving(Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Driving Test", "Tests raw drivetrain control without localization.", true);
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x), true);
        }
        return true;
    }
}

class TestsPose extends TuningOpMode<Boolean> {
    Function<HardwareMap, Localizer> localizerFunction;

    public TestsPose(Function<HardwareMap, Localizer> localizerFunction) {
        super("Pose Test", "Verifies localizer output without a drivetrain.", true);
        this.localizerFunction = localizerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        while (opModeIsActive()) {
            localizer.update();
            telemetry.addData("Pose", localizer.pose());
            telemetry.update();
        }
        return true;
    }
}



/** One 24-inch path followed by an explicit endpoint hold, with no reversal. */
class TestsSingleMove extends TuningOpMode<Boolean> {
    private final Function<HardwareMap, Follower> followerFunction;

    private final boolean strafe;

    TestsSingleMove(Function<HardwareMap, Follower> followerFunction, boolean strafe) {
        super(strafe ? "Single Left 24 Inches" : "Single Forward 24 Inches",
                "Drives 24 inches in the selected direction, then holds the endpoint until STOP. Read Pose while running.", true);
        this.followerFunction = followerFunction;
        this.strafe = strafe;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        Pose target = strafe ? new Pose(0, 24, 0) : new Pose(24, 0, 0);
        follower.setPose(Pose.zero());
        Thread.sleep(1000);
        waitForStart();
        if (isStopRequested()) return false;
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(line(Pose.zero(), target).constant(0));
        ElapsedTime timer = new ElapsedTime();
        boolean holding = false;
        try {
            while (opModeIsActive()) {
                follower.update();
                if (!holding && follower.atParametricEnd()) {
                    follower.hold(target);
                    holding = true;
                }
                telemetry.addData("State", holding ? "Holding endpoint" : (strafe ? "Strafing left" : "Driving forward"));
                telemetry.addData("Pose", follower.pose());
                telemetry.addData("Target", strafe ? "X=0, Y=24 inches, heading=0" : "X=24 inches, Y=0, heading=0");
                telemetry.update();
                if (!holding && timer.seconds() > 12) {
                    telemetry.addLine("Timed out before endpoint; stopping motors.");
                    telemetry.update();
                    return false;
                }
            }
            return holding;
        } finally {
            follower.stop();
        }
    }
}


/** Commands one quarter-turn and keeps holding the target until STOP. */
class TestsSingleTurn extends TuningOpMode<Boolean> {
    private final Function<HardwareMap, Follower> followerFunction;

    TestsSingleTurn(Function<HardwareMap, Follower> followerFunction) {
        super("Single Turn 90 Degrees", "Turns to heading 90 degrees and holds near the starting position until STOP.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        Pose target = new Pose(0, 0, Math.PI / 2);
        follower.setPose(Pose.zero());
        Thread.sleep(1000);
        waitForStart();
        if (isStopRequested()) return false;
        follower.setPose(Pose.zero());
        follower.update();
        follower.hold(target);
        try {
            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.pose();
                telemetry.addData("Pose", pose);
                telemetry.addData("Heading (degrees)", "%.2f", Math.toDegrees(pose.heading()));
                telemetry.addData("Target", "X=0, Y=0, heading=90 degrees (1.571 radians)");
                telemetry.update();
            }
            return true;
        } finally {
            follower.stop();
        }
    }
}


/** A single forward-to-left curve, ending with a quarter-turn and endpoint hold. */
class TestsSingleCurve extends TuningOpMode<Boolean> {
    private final Function<HardwareMap, Follower> followerFunction;

    TestsSingleCurve(Function<HardwareMap, Follower> followerFunction) {
        super("Single Curve 24 by 24 Inches", "Curves 24 inches forward and 24 inches left, then holds at heading 90 degrees until STOP.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        Pose target = new Pose(24, 24, Math.PI / 2);
        follower.setPose(Pose.zero());
        Thread.sleep(1000);
        waitForStart();
        if (isStopRequested()) return false;
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(curve(Pose.zero(), new Pose(24, 0, 0), target).tangent());
        ElapsedTime timer = new ElapsedTime();
        boolean holding = false;
        try {
            while (opModeIsActive()) {
                follower.update();
                if (!holding && follower.atParametricEnd()) {
                    follower.hold(target);
                    holding = true;
                }
                Pose pose = follower.pose();
                telemetry.addData("State", holding ? "Holding endpoint" : "Following curve");
                telemetry.addData("Pose", pose);
                telemetry.addData("Heading (degrees)", "%.2f", Math.toDegrees(pose.heading()));
                telemetry.addData("Target", "X=24 inches, Y=24 inches, heading=90 degrees");
                telemetry.update();
                if (!holding && timer.seconds() > 12) {
                    telemetry.addLine("Timed out before endpoint; stopping motors.");
                    telemetry.update();
                    return false;
                }
            }
            return holding;
        } finally {
            follower.stop();
        }
    }
}
