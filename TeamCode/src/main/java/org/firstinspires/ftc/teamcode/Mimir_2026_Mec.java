package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.dashboard.RobotPanels;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;
import org.firstinspires.ftc.teamcode.mechanisms.ledIndicator;
import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels;
import org.firstinspires.ftc.teamcode.mechanisms.servos_and_shooters;
import org.firstinspires.ftc.teamcode.mechanisms.AimAssist;
import org.firstinspires.ftc.teamcode.mechanisms.AllianceMemory;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.PositionHold;
import org.firstinspires.ftc.teamcode.mechanisms.ShotSolution;
import org.firstinspires.ftc.teamcode.mechanisms.ShotPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.ShotPracticeLog;
import org.firstinspires.ftc.teamcode.mechanisms.SidewaysShotPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.PracticeMovingShotPlanner;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp
@Configurable
public class Mimir_2026_Mec extends OpMode {

    mec_wheels drive = new mec_wheels(); //creates a new drive class
    private final PositionHold positionHold = new PositionHold();
    IMU_setup the_imu = new IMU_setup(); //creates a new imu class
    servos_and_shooters servos = new servos_and_shooters(); //creates a new servo class
    LIMELIGHT_setup the_limelight = new LIMELIGHT_setup(); // creates a new limelight class
    private final AimAssist aimAssist = new AimAssist(the_limelight);
    private final ShotPlanner shotPlanner = new ShotPlanner(aimAssist, servos);
    private final PracticeMovingShotPlanner movingPractice = new PracticeMovingShotPlanner(aimAssist, servos);
    private boolean lastMovingPracticeButton;


    ledIndicator ledIndicator = new ledIndicator();
    private final ElapsedTime hzTimer = new ElapsedTime(); // Timer for frequency tracking

    public void externalTelemetry(String name, double data, boolean telemetryTrigger){
        if (telemetryTrigger){
            telemetry.addData(name, data);
        }
    }

    //movement variables
    double x_axis;
    double y_axis;
    double rotX;
    double rotY;
    double rotation;
    double hz;

    //powers
    double intakeSpeed = -1;
    double indexerSpeed = -0.2; // Change magnitude to adjust indexer servo power.
    double rotationMultiplier = 2;

    //other
    boolean resetYaw;
    //intake

    boolean intakeToggle = false;
    boolean intakeTrigger = false;
    boolean lastIntakeTrigger = false;
    boolean shooterToggle = false;


    //turbo

    boolean turboToggle = false;
    boolean turboTrigger = false;                               //67
    boolean lastTurboTrigger = false;
    boolean lastShooterTrigger = false;
    boolean lastIndexerTrigger = false;
    private boolean holdToggle = false;
    private boolean lastHoldButton = false;
    private boolean aimToggle = false;
    private boolean lastAimButton = false;
    private boolean redAlliance;
    private boolean allianceKnown;
    private boolean shotRumbling = false;
    private boolean practiceMode;
    private boolean lastPracticeButton;
    private boolean lastPracticeLeft;
    private boolean lastPracticeRight;
    private boolean lastSampleButton;
    private double practiceVelocity;
    private final ShotPracticeLog shotLog = new ShotPracticeLog();

        // intake toggle varibles
    private boolean indexerRunning = false;
    // Encoder ticks per second, matching the flywheel tuning OpMode.
    public static volatile double SHOOTER_TARGET_VELOCITY = 1278.0;

    @Override
    public void init() {
        RobotPanels.attach(this);
        loadAlliance();
        //init motors and servos.
        positionHold.init(hardwareMap);
        drive.init(hardwareMap);
        servos.init(hardwareMap); // Added missing initialization
        the_imu.init(hardwareMap); // initialization of IMU
        the_limelight.init(hardwareMap); //initialization of limelight
        ledIndicator.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down && !lastPracticeButton) practiceMode = !practiceMode;
        lastPracticeButton = gamepad1.dpad_down;
        // Optional override for practice or when no colored auto has run in this app session.
        if (gamepad1.dpad_left) {
            AllianceMemory.remember(true);
            loadAlliance();
        } else if (gamepad1.dpad_right) {
            AllianceMemory.remember(false);
            loadAlliance();
        }
        telemetry.addData("Alliance", allianceKnown ? (redAlliance ? "RED" : "BLUE") : "NOT SET");
        telemetry.addLine("Alliance from auto; INIT override: D-pad Left = red, Right = blue");
        telemetry.addData("Shot practice", practiceMode ? "ON: manual calibration shots" : "OFF");
        telemetry.addLine("INIT D-pad Down: toggle practice controls");
        telemetry.addLine("Practice after START: Left/Right speed -/+25; Down saves sample");
        telemetry.addLine("Tagless moving test: at CENTER, manually aim; shooter OFF; click right stick");
        positionHold.updateTracking();
        RobotPanels.pose(positionHold.getPose());
        telemetry.addData("Panels", "Ready: field, telemetry, graphs, joystick inputs and Limelight");
        telemetry.addData("Field origin", "TeleOp INIT pose; set RobotPanels start coordinates before INIT");
        telemetry.addData("Limelight", the_limelight.getStatus());
        telemetry.update();
    }

    @Override
    public void start() {
        the_limelight.limelight_start();
        shotPlanner.reset();
        movingPractice.clear();
        lastMovingPracticeButton = gamepad1.right_stick_button;
        gamepad1.stopRumble();
        shotRumbling = false;
        practiceVelocity = SHOOTER_TARGET_VELOCITY;
        lastPracticeLeft = gamepad1.dpad_left;
        lastPracticeRight = gamepad1.dpad_right;
        lastSampleButton = gamepad1.dpad_down;
        lastAimButton = gamepad1.dpad_up;
        lastHoldButton = gamepad1.left_bumper;
        lastShooterTrigger = gamepad1.y;
        lastIndexerTrigger = gamepad1.b;
    }

    private void loadAlliance() {
        AllianceMemory.Alliance saved = AllianceMemory.get();
        allianceKnown = saved != AllianceMemory.Alliance.UNKNOWN;
        redAlliance = saved == AllianceMemory.Alliance.RED;
    }

    @Override
    public void loop() {
        // Frequency calculation
        hz = 1.0 / hzTimer.seconds();
        hzTimer.reset();

        //setting gamepad variables with deadzone
        x_axis = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;
        y_axis = Math.abs(gamepad1.right_stick_y) > 0.05 ? -gamepad1.right_stick_y : 0;
        rotation = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        resetYaw = gamepad1.back;
        turboTrigger = gamepad1.right_bumper;
        intakeTrigger = gamepad1.a;

        // Toggle once per press.
        if (gamepad1.y && !lastShooterTrigger) {
            shooterToggle = !shooterToggle;
        }
        lastShooterTrigger = gamepad1.y;

        if (gamepad1.b && !lastIndexerTrigger) {
            indexerRunning = !indexerRunning;
        }
        lastIndexerTrigger = gamepad1.b;
        if (!shooterToggle) indexerRunning = false;
        if (practiceMode && !movingPractice.isArmed() && !SidewaysShotPlanner.CALIBRATION.enabled) {
            int step = (gamepad1.dpad_right && !lastPracticeRight ? 1 : 0)
                    - (gamepad1.dpad_left && !lastPracticeLeft ? 1 : 0);
            practiceVelocity = limitShooterVelocity(practiceVelocity + 25.0 * step);
        }
        lastPracticeLeft = gamepad1.dpad_left;
        lastPracticeRight = gamepad1.dpad_right;
        the_limelight.main();
        double shooterVelocity = servos.getShooterVelocity();


        //calculating Mecanum power
        double botHeading = -the_imu.getHeading();
        rotX = x_axis * Math.cos(botHeading) - y_axis * Math.sin(botHeading);
        rotY = x_axis * Math.sin(botHeading) + y_axis * Math.cos(botHeading);

        drive.intakeMode(intakeToggle, intakeSpeed);
        drive.turboMode(turboToggle);
        if (gamepad1.left_bumper && !lastHoldButton) {
            holdToggle = !holdToggle;
            if (holdToggle) aimToggle = false;
        }
        lastHoldButton = gamepad1.left_bumper;

        boolean driverMoving = x_axis != 0 || y_axis != 0 || rotation != 0;

        if (driverMoving) {
            holdToggle = false;
        }
        if (gamepad1.dpad_up && !lastAimButton) {
            aimToggle = !aimToggle;
            if (aimToggle) holdToggle = false;
        }
        lastAimButton = gamepad1.dpad_up;

        boolean aiming = aimToggle && allianceKnown;

        if (aiming) {
            holdToggle = false;
            positionHold.stopHolding();
            positionHold.updateTracking();
            drive.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (holdToggle) {
            drive.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            positionHold.startHolding();
            positionHold.update();
        } else {
            positionHold.stopHolding();
            positionHold.updateTracking();
            drive.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        servos.servos_move(intakeToggle, intakeSpeed);
        if (practiceMode && gamepad1.right_stick_button && !lastMovingPracticeButton) {
            if (movingPractice.isArmed()) {
                movingPractice.clear();
                indexerRunning = false;
            } else if (!aimToggle && !shooterToggle && !holdToggle && allianceKnown) {
                movingPractice.capture(positionHold.getPose(), positionHold.getVelocity());
                if (movingPractice.isArmed()) practiceVelocity = PracticeMovingShotPlanner.TARGET_SPEED;
            }
        }
        lastMovingPracticeButton = gamepad1.right_stick_button;
        double requestedVelocity = practiceMode ? practiceVelocity : SHOOTER_TARGET_VELOCITY;
        requestedVelocity = shotPlanner.requestedVelocity(
                redAlliance, requestedVelocity, practiceMode);
        if (practiceMode && movingPractice.isArmed()) requestedVelocity = PracticeMovingShotPlanner.TARGET_SPEED;
        requestedVelocity = limitShooterVelocity(requestedVelocity);
        servos.shooterthingy(shooterToggle, requestedVelocity);
        boolean movingTest = practiceMode && movingPractice.isArmed();
        boolean sidewaysMode = SidewaysShotPlanner.CALIBRATION.enabled || movingTest;
        if (!aiming) shotPlanner.resetSideways();
        if (!aiming) movingPractice.resetMotion();
        ShotSolution shot = new ShotSolution(0, requestedVelocity, false, "Alliance not set");
        boolean smartShotReady = false;
        if (allianceKnown) {
            shot = aiming ? shotPlanner.evaluate(redAlliance, positionHold.getPose(), positionHold.getVelocity())
                    : shotPlanner.evaluate(redAlliance, positionHold.getVelocity());
            if (movingTest && aiming) shot = movingPractice.evaluate(
                    positionHold.getPose(), positionHold.getVelocity());
            telemetry.addData("shot status",shot.status);
            telemetry.addData("shot ready",shot.ready);
            smartShotReady = shot.ready && (!practiceMode || sidewaysMode);
            telemetry.addData("speed (in/s)",Math.hypot(
                    positionHold.getVelocity().vx,
                    positionHold.getVelocity().vy));
            telemetry.addData("turn speed (deg/s)",Math.toDegrees(
                    positionHold.getVelocity().omega));

        } else {
            telemetry.addData("shot status", "Alliance not set");
        }

        // Steering, feed permission and rumble consume the SAME solution.
        if (aiming) drive.main(rotX, rotY, aimAssist.trackingTurn(shot.turn, indexerRunning), 1.0);
        else if (!holdToggle) drive.main(rotX, rotY, rotation, rotationMultiplier);

        boolean shouldRumble = aiming && smartShotReady;

        if (shouldRumble != shotRumbling) {
            if (shouldRumble) {
                gamepad1.rumble(1.0, 1.0,
                        Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
                gamepad1.stopRumble();
            }
            shotRumbling = shouldRumble;
        }
        // Holding X overrides the forward toggle; release to restore its state.
        boolean practiceFeedReady = practiceMode && !movingTest && aiming
                && shotPlanner.practiceFeedReady(redAlliance);
        boolean feedAllowed = indexerRunning && shooterToggle && servos.isShooterReady()
                && (movingTest ? aiming && smartShotReady : (!aimToggle || smartShotReady || practiceFeedReady));

        servos.indexerthingy(feedAllowed || gamepad1.x,
                gamepad1.x ? -indexerSpeed : indexerSpeed);

        if (practiceMode && gamepad1.dpad_down && !lastSampleButton) {
            shotLog.record(allianceKnown, redAlliance, aimAssist,
                    servos.getShooterTargetVelocity(), shooterVelocity, batteryVoltage(),
                    positionHold.getPose(), positionHold.getVelocity(), aiming,
                    feedAllowed && !gamepad1.x);
        }
        lastSampleButton = gamepad1.dpad_down;
        if (practiceMode) {
            telemetry.addLine("PRACTICE: manual shots; calibration is NOT validated");
            telemetry.addLine("D-pad L/R: speed -/+25 ticks/s; Down: save snapshot");
            telemetry.addData("Practice velocity setting", practiceVelocity);
            telemetry.addData("Practice aim feed", practiceFeedReady ? "Allowed" : "Wait: tag, aim, 97%, still 2s");
            telemetry.addData("Saved samples", shotLog.getSamples());
            telemetry.addData("Sample status", shotLog.getStatus());
            telemetry.addData("Sample file", shotLog.getPath());
        }
        telemetry.addData("Sideways profile", sidewaysMode ? shot.status : "Disabled: measurements needed");

        if (resetYaw) {
            the_imu.resetYaw();
        }



        ledIndicator.indicate(shooterToggle, shooterVelocity, requestedVelocity);
        RobotPanels.shooter(shooterToggle, shooterVelocity, requestedVelocity);
        RobotPanels.pose(positionHold.getPose());
        telemetry.addData("Drive strafe command", x_axis);
        telemetry.addData("Drive forward command", y_axis);
        telemetry.addData("Drive turn command", rotation);
        telemetry.addData("Field relative strafe command", rotX);
        telemetry.addData("Field relative forward command", rotY);
        telemetry.addData("Position hold", holdToggle);
        telemetry.addData("Alliance", allianceKnown ? (redAlliance ? "RED" : "BLUE") : "NOT SET");
        telemetry.addData("Auto aim", !allianceKnown ? "Unavailable: select alliance during INIT"
                : aiming ? "ON" : "OFF");
        telemetry.addData("Shooter error (ticks/s)",
                (shooterToggle ? requestedVelocity : 0) - shooterVelocity);

        // Measured wheel speeds from the motor encoders
        telemetry.addData("Gamepad1 A" , intakeTrigger);
        telemetry.addData("Turbo Mode?" , turboToggle);
        telemetry.addData("Intake Mode?" , intakeToggle);
        telemetry.addData("Shooter target (ticks/s)", shooterToggle ? requestedVelocity : 0.0);
        telemetry.addData("Shooter velocity (ticks/s)", "%.0f", shooterVelocity);
        telemetry.addData("Shooter", shooterToggle ? "On" : "Off");
        telemetry.addData("Indexer request", indexerRunning ? "ON" : "OFF");
        telemetry.addData("Indexer", gamepad1.x ? "Reverse" : feedAllowed ? "Feeding" : indexerRunning ? "Waiting" : "Off");
        telemetry.addData("Battery (V)", batteryVoltage());
        telemetry.addData("Flywheel feed check", servos.getShooterFeedStatus());
        telemetry.addData("Flywheel speed drops", servos.getFlywheelSpeedDrops());
        telemetry.addData("Heading (Degrees)" , the_imu.getHeading() * (180/Math.PI)); //shows the heading of the robot
        telemetry.addData("Loop Frequency (Hz)", hz);
        telemetry.addData("Camera status", the_limelight.getStatus());
        telemetry.addData("Camera connected", the_limelight.isConnected());
        telemetry.addData("Camera pipeline", the_limelight.getPipelineIndex());
        telemetry.addData("Camera data age", the_limelight.getResultAgeMs());
        externalTelemetry("Target X", the_limelight.returnLLResultTx(), the_limelight.txTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Y", the_limelight.returnLLResultTy(), the_limelight.tyTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Area", the_limelight.returnLLResultTa(), the_limelight.taTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Tag ID", the_limelight.returnTagId(), the_limelight.tagIdTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.

        if (allianceKnown && aimAssist.hasAimTarget(redAlliance)) {
            telemetry.addData("Selected aim tag",
                    aimAssist.getAimTagId(redAlliance));
            telemetry.addData("Aim error (deg)",
                    aimAssist.getAimError(redAlliance));
            telemetry.addData("Raw aim TY (deg)", aimAssist.getRawAimError(redAlliance));
            telemetry.addData("Camera-tag range (in)",
                    aimAssist.getTagRangeInches(redAlliance));
            telemetry.addData("Range status", aimAssist.getTagRangeReading(redAlliance).status);
            telemetry.addData("Aim correction",
                    aimAssist.getTurn(redAlliance));
        } else {
            telemetry.addData("Aim assist", "No fresh alliance tag");
        }

        if (practiceMode) {
            // A compact DS screen keeps the sample result and essential measurements visible.
            telemetry.clear();
            telemetry.addData("Moving test", movingPractice.referenceStatus());
            telemetry.addData("Moving test status", movingTest ? shot.status : "OFF: Y + aim OFF; click right stick at CENTER");
            telemetry.addData("FEED CHECK", !indexerRunning ? "Press B to request feeding"
                    : !shooterToggle ? "Press Y: shooter ON"
                    : movingTest && !aiming ? "D-pad Up: auto aim ON"
                    : !servos.isShooterReady() ? servos.getShooterFeedStatus()
                    : feedAllowed ? "INDEXING" : shot.status);
            telemetry.addData("Sample status", shotLog.getStatus());
            telemetry.addData("Saved samples", shotLog.getSamples());
            telemetry.addData("Alliance / aim", (allianceKnown ? (redAlliance ? "RED" : "BLUE") : "NOT SET")
                    + " / " + (aiming ? "ON" : "OFF"));
            telemetry.addData("Selected aim tag", allianceKnown ? aimAssist.getAimTagId(redAlliance) : -1);
            AimAssist.RangeReading rangeReading = aimAssist.getTagRangeReading(redAlliance);
            telemetry.addData("Camera-tag range (in)", allianceKnown ? rangeReading.inches : Double.NaN);
            telemetry.addData("Range status", allianceKnown ? rangeReading.status : "Select alliance during INIT");
            telemetry.addData("Camera status", the_limelight.getStatus());
            telemetry.addData("Target / actual ticks/s", "%.0f / %.0f", shooterToggle ? requestedVelocity : 0, shooterVelocity);
            telemetry.addData("Flywheel feed check", servos.getShooterFeedStatus());
            telemetry.addData("Flywheel speed drops", servos.getFlywheelSpeedDrops());
            telemetry.addData("Aim error / lead deg", "%.2f / %.2f", shot.aimErrorDegrees, shot.leadDegrees);
            telemetry.addData("Robot vx / vy / turn", "%.2f / %.2f / %.2f deg/s", positionHold.getVelocity().vx,
                    positionHold.getVelocity().vy, Math.toDegrees(positionHold.getVelocity().omega));
            telemetry.addData("Heading (deg)", Math.toDegrees(positionHold.getPose().heading()));
            telemetry.addData("Raw TY (deg)", allianceKnown ? aimAssist.getRawAimError(redAlliance) : Double.NaN);
            telemetry.addData("Shot status", shot.status);
            telemetry.addData("Indexer", gamepad1.x ? "REVERSE" : feedAllowed ? "FEEDING" : indexerRunning ? "WAITING" : "OFF");
            telemetry.addData("Battery (V)", batteryVoltage());
            telemetry.addData("Controls", sidewaysMode ? "Fixed speed; Down saves" : "L/R speed; Down saves; Up aim");
        }
        telemetry.update();

        if (turboTrigger && !lastTurboTrigger) {
            turboToggle = !turboToggle;
            }
        lastTurboTrigger = turboTrigger;

        if (intakeTrigger && !lastIntakeTrigger) {
            intakeToggle = !intakeToggle;
        }
        lastIntakeTrigger = intakeTrigger;
    }

    private double limitShooterVelocity(double requested) {
        double maximum = servos.getMaxShooterVelocity();
        if (!Double.isFinite(requested) || !Double.isFinite(maximum) || maximum <= 0) return 0;
        return Math.max(0, Math.min(maximum, requested));
    }

    private double batteryVoltage() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && Double.isFinite(voltage)) lowest = Math.min(lowest, voltage);
        }
        return Double.isFinite(lowest) ? lowest : Double.NaN;
    }

    @Override
    public void stop() {
        movingPractice.clear();
        positionHold.stopHolding();
        the_limelight.limelight_stop();
        shooterToggle = false;
        indexerRunning = false;
        gamepad1.stopRumble();
        shotRumbling = false;
        servos.shooterthingy(false, SHOOTER_TARGET_VELOCITY);
        servos.indexerthingy(false, indexerSpeed);
        servos.servos_move(false, intakeSpeed);
        RobotPanels.stopped();
    }

}
