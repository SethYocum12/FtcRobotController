package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dashboard.RobotPanels;
import org.firstinspires.ftc.teamcode.mechanisms.AllianceMemory;
import org.firstinspires.ftc.teamcode.mechanisms.AutoAimTag;
import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.PositionHold;
import org.firstinspires.ftc.teamcode.mechanisms.ledIndicator;
import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels;
import org.firstinspires.ftc.teamcode.mechanisms.servos_and_shooters;

@TeleOp
@Configurable
public class Mimir_2026_Mec extends OpMode {

    private final mec_wheels drive = new mec_wheels();
    private final PositionHold positionHold = new PositionHold();
    private final AutoAimTag autoAim = new AutoAimTag();
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

    // Manually adjustable target used by the flywheel.
    public static volatile double SHOOTER_TARGET_VELOCITY = 1278.0;

    @Override
    public void init() {
        RobotPanels.attach(this);
        loadAlliance();
        positionHold.init(hardwareMap);
        autoAim.init(hardwareMap, this::getRuntime);
        drive.init(hardwareMap);
        servos.init(hardwareMap);
        imu.init(hardwareMap);
        led.init(hardwareMap);
    }

    @Override
    public void init_loop() {
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
        telemetry.addLine("Hold Left Trigger after START for AprilTag auto alignment");
        telemetry.addData("Shooter speed practice", practiceMode ? "ON" : "OFF");
        telemetry.addLine("INIT D-pad Down toggles practice speed control");
        positionHold.updateTracking();
        RobotPanels.pose(positionHold.getPose());
        telemetry.addData("Panels", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        autoAim.start();
        practiceVelocity = SHOOTER_TARGET_VELOCITY;
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
        if (gamepad1.left_trigger > 0.3) holdToggle = false;

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

        double driveRotation = autoAim.getTurn(redAlliance, allianceKnown, rotation, gamepad1);
        if (!holdToggle) drive.main(fieldX, fieldY, driveRotation, ROTATION_MULTIPLIER);
        autoAim.tune(gamepad2);

        boolean feedAllowed = indexerRunning && shooterToggle;
        servos.indexerthingy(feedAllowed || gamepad1.x,
                gamepad1.x ? -INDEXER_SPEED : INDEXER_SPEED);

        if (gamepad1.back) imu.resetYaw();

        double shooterVelocity = servos.getShooterVelocity();
        led.indicate(shooterToggle, shooterVelocity, requestedVelocity);
        RobotPanels.shooter(shooterToggle, shooterVelocity, requestedVelocity);
        RobotPanels.pose(positionHold.getPose());

        telemetry.addData("Alliance", allianceKnown ? (redAlliance ? "RED" : "BLUE") : "NOT SET");
        telemetry.addData("Practice speed", practiceMode ? practiceVelocity : "OFF");
        telemetry.addData("Drive strafe", xAxis);
        telemetry.addData("Drive forward", yAxis);
        telemetry.addData("Drive turn", rotation);
        autoAim.addTelemetry(telemetry);
        if (!allianceKnown) telemetry.addLine("Set alliance in INIT before enabling auto aim");
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
        autoAim.stop();
        shooterToggle = false;
        indexerRunning = false;
        servos.shooterthingy(false, SHOOTER_TARGET_VELOCITY);
        servos.indexerthingy(false, INDEXER_SPEED);
        servos.servos_move(false, INTAKE_SPEED);
        RobotPanels.stopped();
    }
}
