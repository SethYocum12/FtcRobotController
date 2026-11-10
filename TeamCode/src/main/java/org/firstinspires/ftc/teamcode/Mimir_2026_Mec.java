package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dashboard.RobotPanels;
import org.firstinspires.ftc.teamcode.mechanisms.AllianceMemory;
import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;
import org.firstinspires.ftc.teamcode.mechanisms.PositionHold;
import org.firstinspires.ftc.teamcode.mechanisms.StationaryShotCue;
import org.firstinspires.ftc.teamcode.mechanisms.StationaryShotModel;
import org.firstinspires.ftc.teamcode.mechanisms.ShotMathPracticeLog;
import org.firstinspires.ftc.teamcode.mechanisms.ShotTrajectoryMath;
import org.firstinspires.ftc.teamcode.mechanisms.ledIndicator;
import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels;
import org.firstinspires.ftc.teamcode.mechanisms.servos_and_shooters;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;

@TeleOp
@Configurable
public class Mimir_2026_Mec extends OpMode {

    private final mec_wheels drive = new mec_wheels();
    private final PositionHold positionHold = new PositionHold();
    private final LIMELIGHT_setup vision = new LIMELIGHT_setup();
    private final StationaryShotCue shotCue = new StationaryShotCue();
    private final StationaryShotModel shotModel = new StationaryShotModel();
    private final ShotMathPracticeLog shotPractice = new ShotMathPracticeLog();
    private final IMU_setup imu = new IMU_setup();
    private final servos_and_shooters servos = new servos_and_shooters();
    private final ledIndicator led = new ledIndicator();
    private final ElapsedTime hzTimer = new ElapsedTime();

    private double xAxis;
    private double yAxis;
    private double fieldX;
    private double fieldY;
    private double rotation;
    private double hz;

    private static final double INTAKE_SPEED = -1.0;
    private static final double INDEXER_SPEED = -0.2;
    private static final double ROTATION_MULTIPLIER = 2.0;

    private boolean intakeToggle;
    private boolean lastIntakeButton;
    private boolean shooterToggle;
    private boolean lastShooterButton;
    private boolean indexerRunning;
    private boolean lastIndexerButton;
    private boolean turboToggle;
    private boolean lastTurboButton;
    private boolean holdToggle;
    private boolean lastHoldButton;
    private boolean practiceMode;
    private boolean lastPracticeButton;
    private boolean lastPracticeLeft;
    private boolean lastPracticeRight;
    private boolean redAlliance;
    private boolean allianceKnown;
    private double practiceVelocity;
    private boolean lastConfirmShotButton;
    private boolean lastPracticeMarkerButton;
    private boolean cueRumbling;
    private double lastRumbleSeconds = Double.NEGATIVE_INFINITY;
    private double clearCueSince = Double.NaN;
    private boolean clearCueHandled;

    // Manually adjustable target used by the flywheel.
    public static volatile double SHOOTER_TARGET_VELOCITY = 1325.0;

    @Override
    public void init() {
        RobotPanels.attach(this);
        loadAlliance();
        positionHold.init(hardwareMap);
        vision.init(hardwareMap);
        shotCue.load(AppUtil.getInstance().getSettingsFile("mimir-biobuzz-hive-shot-cue.properties"));
        shotModel.load(AppUtil.getInstance().getSettingsFile("mimir-stationary-shot-math.properties"));
        gamepad1.stopRumble();
        drive.init(hardwareMap);
        servos.init(hardwareMap);
        imu.init(hardwareMap);
        led.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        // Explicitly clear only shot references when camera/shooter setup changes.
        if (gamepad1.x && gamepad1.y) {
            if (!Double.isFinite(clearCueSince)) clearCueSince = getRuntime();
            if (!clearCueHandled && getRuntime() - clearCueSince >= 1.0) {
                shotCue.clear();
                shotModel.invalidate();
                clearCueHandled = true;
            }
        } else {
            clearCueSince = Double.NaN;
            clearCueHandled = false;
        }
        if (gamepad1.dpad_down && !lastPracticeButton) {
            practiceMode = !practiceMode;
        }
        lastPracticeButton = gamepad1.dpad_down;

        // Alliance selection and memory are kept for the autonomous routines.
        if (gamepad1.dpad_left) {
            AllianceMemory.remember(true);
            loadAlliance();
        } else if (gamepad1.dpad_right) {
            AllianceMemory.remember(false);
            loadAlliance();
        }

        telemetry.addData("Alliance", allianceKnown ? (redAlliance ? "RED" : "BLUE") : "NOT SET");
        telemetry.addLine("INIT D-pad Left = red; D-pad Right = blue");
        telemetry.addLine("Shot rumble is automatic; D-pad Up confirms a successful shooting spot");
        telemetry.addData("Saved shots", shotCue.getSaveStatus());
        telemetry.addData("Shot math", shotModel.getStatus());
        telemetry.addLine("INIT Square + Triangle for 1 second clears shot calibration");
        telemetry.addData("Shooter speed practice", practiceMode ? "ON" : "OFF");
        telemetry.addLine("INIT Down: practice recording; after START Down labels each position");
        positionHold.updateTracking();
        RobotPanels.pose(positionHold.getPose());
        telemetry.addData("Panels", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        vision.limelight_start();
        shotCue.reset();
        cueRumbling = false;
        gamepad1.stopRumble();
        lastRumbleSeconds = Double.NEGATIVE_INFINITY;
        lastConfirmShotButton = gamepad1.dpad_up;
        lastPracticeMarkerButton = gamepad1.dpad_down;
        practiceVelocity = SHOOTER_TARGET_VELOCITY;
        if (practiceMode) shotPractice.start(AppUtil.getInstance().getSettingsFile("mimir-shot-math"));
        lastPracticeLeft = gamepad1.dpad_left;
        lastPracticeRight = gamepad1.dpad_right;
        lastIntakeButton = gamepad1.a;
        lastShooterButton = gamepad1.y;
        lastIndexerButton = gamepad1.b;
        lastTurboButton = gamepad1.right_bumper;
        lastHoldButton = gamepad1.left_bumper;
    }

    private void loadAlliance() {
        AllianceMemory.Alliance saved = AllianceMemory.get();
        allianceKnown = saved != AllianceMemory.Alliance.UNKNOWN;
        redAlliance = saved == AllianceMemory.Alliance.RED;
    }

    @Override
    public void loop() {
        hz = 1.0 / hzTimer.seconds();
        hzTimer.reset();

        xAxis = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;
        yAxis = Math.abs(gamepad1.right_stick_y) > 0.05 ? -gamepad1.right_stick_y : 0;
        rotation = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;

        if (gamepad1.y && !lastShooterButton) shooterToggle = !shooterToggle;
        lastShooterButton = gamepad1.y;

        if (gamepad1.b && !lastIndexerButton) indexerRunning = !indexerRunning;
        lastIndexerButton = gamepad1.b;
        if (!shooterToggle) indexerRunning = false;

        if (practiceMode) {
            int step = (gamepad1.dpad_right && !lastPracticeRight ? 1 : 0)
                    - (gamepad1.dpad_left && !lastPracticeLeft ? 1 : 0);
            practiceVelocity = limitShooterVelocity(practiceVelocity + 25.0 * step);
        }
        lastPracticeLeft = gamepad1.dpad_left;
        lastPracticeRight = gamepad1.dpad_right;

        double botHeading = -imu.getHeading();
        fieldX = xAxis * Math.cos(botHeading) - yAxis * Math.sin(botHeading);
        fieldY = xAxis * Math.sin(botHeading) + yAxis * Math.cos(botHeading);

        if (gamepad1.a && !lastIntakeButton) intakeToggle = !intakeToggle;
        lastIntakeButton = gamepad1.a;
        if (gamepad1.right_bumper && !lastTurboButton) turboToggle = !turboToggle;
        lastTurboButton = gamepad1.right_bumper;
        if (gamepad1.left_bumper && !lastHoldButton) holdToggle = !holdToggle;
        lastHoldButton = gamepad1.left_bumper;

        drive.intakeMode(intakeToggle, INTAKE_SPEED);
        drive.turboMode(turboToggle);

        boolean driverMoving = xAxis != 0 || yAxis != 0 || rotation != 0;
        if (driverMoving) holdToggle = false;

        if (holdToggle) {
            drive.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            positionHold.startHolding();
            positionHold.update();
        } else {
            positionHold.stopHolding();
            positionHold.updateTracking();
            drive.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        servos.servos_move(intakeToggle, INTAKE_SPEED);
        double requestedVelocity = limitShooterVelocity(
                practiceMode ? practiceVelocity : SHOOTER_TARGET_VELOCITY);
        servos.shooterthingy(shooterToggle, requestedVelocity);

        if (!holdToggle) drive.main(fieldX, fieldY, rotation, ROTATION_MULTIPLIER);

        boolean feedAllowed = indexerRunning && shooterToggle;
        servos.indexerthingy(feedAllowed || gamepad1.x,
                gamepad1.x ? -INDEXER_SPEED : INDEXER_SPEED);

        if (gamepad1.back) imu.resetYaw();

        double shooterVelocity = servos.getShooterVelocity();
        vision.main();
        StationaryShotCue.Reading cueReading = shotCueReading(requestedVelocity, driverMoving,
                feedAllowed || gamepad1.x);
        boolean savedReady = shotCue.update(cueReading);
        boolean predictedFit = shotModel.update(cueReading);
        boolean cueReady = shotModel.isCalibrated()
                ? shotCue.isOperatingReady() && predictedFit : savedReady;
        String cueStatus = shotModel.isCalibrated()
                ? (shotCue.isOperatingReady() ? shotModel.getStatus() : shotCue.getOperatingStatus())
                : shotCue.getStatus();
        if (practiceMode && gamepad1.dpad_down && !lastPracticeMarkerButton) {
            shotPractice.markPosition(cueReading);
        }
        lastPracticeMarkerButton = gamepad1.dpad_down;
        if (gamepad1.dpad_up && !lastConfirmShotButton) {
            if (shotCue.confirmScoringPosition(cueReading)) shotPractice.markConfirmed(cueReading);
        }
        lastConfirmShotButton = gamepad1.dpad_up;
        shotPractice.sample(cueReading);
        double rumbleSeconds = getRuntime();
        boolean cueFresh = rumbleSeconds >= cueReading.seconds
                && rumbleSeconds - cueReading.seconds <= 0.25;
        updateShotRumble(cueReady && cueFresh, rumbleSeconds);

        led.indicate(shooterToggle, shooterVelocity, requestedVelocity);
        RobotPanels.shooter(shooterToggle, shooterVelocity, requestedVelocity);
        RobotPanels.pose(positionHold.getPose());

        if (practiceMode) {
            int posedTags = 0;
            for (StationaryShotCue.Target target : cueReading.targets) {
                if (target.hasCameraPose()) posedTags++;
            }
            telemetry.addData("PRACTICE POSITION", shotPractice.getBatch());
            telemetry.addData("3D DATA", posedTags > 0 ? "READY: " + posedTags + " tags" : "WAIT: fresh full 3D tag data missing");
            telemetry.addData("FLYWHEEL", servos.getShooterFeedStatus());
            telemetry.addData("Test target / actual ticks/s", "%.0f / %.0f", requestedVelocity, shooterVelocity);
            telemetry.addData("Recording", shotPractice.getStatus());
            telemetry.addData("CSV file", shotPractice.getPath());
            telemetry.addLine("After START: Down labels the next position; film and count your shot set");
        }
        telemetry.addData("SHOT CUE", cueStatus);
        telemetry.addData("Cue source", shotModel.isCalibrated() ? "Trajectory prediction" : "Saved spot; prediction not trained");
        telemetry.addData("Shot math", shotModel.getStatus());
        ShotTrajectoryMath.Prediction prediction = shotModel.getPrediction();
        if (prediction != null) {
            telemetry.addData("Predicted time / clearance", "%.3f s / %.2f in",
                    prediction.flightSeconds, prediction.clearanceInches);
        }
        telemetry.addData("Saved shots", shotCue.getSaveStatus());
        telemetry.addData("Saved tag references", shotCue.getReferenceCount());
        telemetry.addData("Vision", vision.getStatus());
        if (!cueReading.targets.isEmpty()) {
            StationaryShotCue.Target target = cueReading.targets.get(0);
            telemetry.addData("Tag / TX / TY / range (in)", "%d / %.2f / %.2f / %.1f",
                    target.id, target.tx, target.ty, target.rangeInches);
        }
        telemetry.addData("Motion (in/s, deg/s)", "%.2f / %.2f",
                Math.hypot(cueReading.vx, cueReading.vy), Math.toDegrees(cueReading.omega));
        telemetry.addLine("Automatic shot cue; Up confirms a spot AFTER scoring, without moving");
        telemetry.addData("Alliance", allianceKnown ? (redAlliance ? "RED" : "BLUE") : "NOT SET");
        telemetry.addData("Practice speed", practiceMode ? practiceVelocity : "OFF");
        telemetry.addData("Drive strafe", xAxis);
        telemetry.addData("Drive forward", yAxis);
        telemetry.addData("Drive turn", rotation);
        if (!allianceKnown) telemetry.addLine("Set alliance in INIT for the shot cue");
        telemetry.addData("Position hold", holdToggle);
        telemetry.addData("Shooter", shooterToggle ? "ON" : "OFF");
        telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f",
                shooterToggle ? requestedVelocity : 0.0, shooterVelocity);
        telemetry.addData("Indexer", gamepad1.x ? "REVERSE" : feedAllowed ? "MANUAL ON" : "OFF");
        telemetry.addData("Flywheel", servos.getShooterFeedStatus());
        telemetry.addData("Heading (degrees)", Math.toDegrees(imu.getHeading()));
        telemetry.addData("Loop frequency (Hz)", hz);
        telemetry.addData("Battery (V)", batteryVoltage());

        if (practiceMode) {
            telemetry.addLine("D-pad Left/Right changes flywheel target by 25 ticks/s");
            telemetry.addLine("B toggles indexer; X reverses; Y toggles shooter");
        }
        telemetry.update();
    }

    private StationaryShotCue.Reading shotCueReading(double requestedVelocity,
                                                     boolean driverMoving, boolean feeding) {
        StationaryShotCue.Reading reading = new StationaryShotCue.Reading();
        reading.seconds = getRuntime();
        reading.allianceKnown = allianceKnown;
        reading.redAlliance = redAlliance;
        reading.driveRequested = driverMoving;
        reading.shooterOn = shooterToggle;
        reading.targetVelocity = requestedVelocity;
        reading.actualVelocity = servos.getShooterVelocity();
        reading.batteryVoltage = batteryVoltage();
        reading.visionAgeMs = vision.getResultAgeMs();
        reading.flywheelReady = servos.isShooterReady();
        reading.feeding = feeding;
        reading.reversing = gamepad1.x;
        reading.robotX = positionHold.getPose().x();
        reading.robotY = positionHold.getPose().y();
        reading.robotHeading = positionHold.getPose().heading();
        Velocity velocity = positionHold.getVelocity();
        if (velocity != null) {
            reading.vx = velocity.vx;
            reading.vy = velocity.vy;
            reading.omega = velocity.omega;
        }
        reading.targets = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : vision.getFreshFiducials()) {
            int id = tag.getFiducialId();
            if (id < (redAlliance ? 30 : 38) || id > (redAlliance ? 37 : 45)) continue;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            double range = poseRangeInches(pose);
            if (!Double.isFinite(range)) range = poseRangeInches(tag.getCameraPoseTargetSpace());
            double yaw = Double.NaN, pitch = Double.NaN, roll = Double.NaN;
            double cameraX = Double.NaN, cameraY = Double.NaN, cameraZ = Double.NaN;
            if (pose != null && pose.getPosition() != null) {
                Position cameraPosition = pose.getPosition().toUnit(DistanceUnit.INCH);
                cameraX = cameraPosition.x;
                cameraY = cameraPosition.y;
                cameraZ = cameraPosition.z;
            }
            if (pose != null && pose.getOrientation() != null) {
                yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                pitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
                roll = pose.getOrientation().getRoll(AngleUnit.DEGREES);
            }
            reading.targets.add(new StationaryShotCue.Target(id,
                    tag.getTargetXDegrees(), tag.getTargetYDegrees(), range, yaw, pitch, roll,
                    cameraX, cameraY, cameraZ));
        }
        return reading;
    }

    private static double poseRangeInches(Pose3D pose) {
        if (pose == null || pose.getPosition() == null) return Double.NaN;
        Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
        double range = Math.hypot(Math.hypot(position.x, position.y), position.z);
        return Double.isFinite(range) && range > 0 ? range : Double.NaN;
    }

    private void updateShotRumble(boolean ready, double seconds) {
        // Overlapping short rumbles feel continuous and expire if the loop stops updating.
        if (ready) {
            if (!cueRumbling || seconds - lastRumbleSeconds >= 0.1) {
                gamepad1.rumble(0.35, 0.35, 200);
                lastRumbleSeconds = seconds;
            }
            cueRumbling = true;
        } else if (cueRumbling) {
            gamepad1.stopRumble();
            cueRumbling = false;
        }
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
        positionHold.stopHolding();
        vision.limelight_stop();
        shotCue.reset();
        gamepad1.stopRumble();
        cueRumbling = false;
        shooterToggle = false;
        indexerRunning = false;
        servos.shooterthingy(false, SHOOTER_TARGET_VELOCITY);
        servos.indexerthingy(false, INDEXER_SPEED);
        servos.servos_move(false, INTAKE_SPEED);
        shotPractice.finish();
        RobotPanels.stopped();
    }
}
