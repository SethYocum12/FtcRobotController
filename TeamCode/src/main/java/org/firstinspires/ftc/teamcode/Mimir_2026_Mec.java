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
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.PositionHold;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Configurable
public class Mimir_2026_Mec extends OpMode {

    mec_wheels drive = new mec_wheels(); //creates a new drive class
    private final PositionHold positionHold = new PositionHold();
    IMU_setup the_imu = new IMU_setup(); //creates a new imu class
    servos_and_shooters servos = new servos_and_shooters(); //creates a new servo class
    LIMELIGHT_setup the_limelight = new LIMELIGHT_setup(); // creates a new limelight class
    private final AimAssist aimAssist = new AimAssist(the_limelight);


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
    private static final boolean RED_ALLIANCE = true;

        // intake toggle varibles
    private boolean indexerRunning = false;
    // Encoder ticks per second, matching the flywheel tuning OpMode.
    public static volatile double SHOOTER_TARGET_VELOCITY = 1278.0;

    @Override
    public void init() {
        RobotPanels.attach(this);
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
        }
        lastHoldButton = gamepad1.left_bumper;

        boolean driverMoving = x_axis != 0 || y_axis != 0 || rotation != 0;

        if (driverMoving) {
            holdToggle = false;
        }
        boolean aiming = gamepad1.dpad_up && !driverMoving;
        if (aiming) {
            holdToggle = false;
            positionHold.stopHolding();
            positionHold.updateTracking();
            drive.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.main(0, 0, aimAssist.getTurn(RED_ALLIANCE), 1.0);
        } else if (holdToggle) {
            drive.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            positionHold.startHolding();
            positionHold.update();
        } else {
            positionHold.stopHolding();
            positionHold.updateTracking();
            drive.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.main(rotX, rotY, rotation, rotationMultiplier);
        }
        servos.servos_move(intakeToggle, intakeSpeed);
        servos.shooterthingy(shooterToggle, SHOOTER_TARGET_VELOCITY);
        // Holding X overrides the forward toggle; release to restore its state.
        servos.indexerthingy(indexerRunning || gamepad1.x,
                gamepad1.x ? -indexerSpeed : indexerSpeed);
        if (the_limelight.txTele()) {
            double errorDegrees = the_limelight.returnLLResultTy();
            telemetry.addData("Aim error", errorDegrees);
            telemetry.addData("Aim correction preview",
                    aimAssist.calculateTurn(errorDegrees));
        } else {
            telemetry.addData("Aim assist", "No fresh tag"); }

        if (resetYaw) {
            the_imu.resetYaw();
        }



        ledIndicator.indicate(shooterToggle, shooterVelocity, SHOOTER_TARGET_VELOCITY);
        RobotPanels.shooter(shooterToggle, shooterVelocity, SHOOTER_TARGET_VELOCITY);
        RobotPanels.pose(positionHold.getPose());
        telemetry.addData("Drive strafe command", x_axis);
        telemetry.addData("Drive forward command", y_axis);
        telemetry.addData("Drive turn command", rotation);
        telemetry.addData("Field relative strafe command", rotX);
        telemetry.addData("Field relative forward command", rotY);
        telemetry.addData("Position hold", holdToggle);
        telemetry.addData("Shooter error (ticks/s)",
                (shooterToggle ? SHOOTER_TARGET_VELOCITY : 0) - shooterVelocity);

        // Measured wheel speeds from the motor encoders
        telemetry.addData("Gamepad1 A" , intakeTrigger);
        telemetry.addData("Turbo Mode?" , turboToggle);
        telemetry.addData("Intake Mode?" , intakeToggle);
        telemetry.addData("Shooter target (ticks/s)", shooterToggle ? SHOOTER_TARGET_VELOCITY : 0.0);
        telemetry.addData("Shooter velocity (ticks/s)", "%.0f", shooterVelocity);
        telemetry.addData("Shooter", shooterToggle ? "On" : "Off");
        telemetry.addData("Indexer", gamepad1.x ? "Reverse" : indexerRunning ? "On" : "Off");
        telemetry.addData("Heading (Degrees)" , the_imu.getHeading() * (180/Math.PI)); //shows the heading of the robot
        telemetry.addData("Loop Frequency (Hz)", hz);
        externalTelemetry("Target X", the_limelight.returnLLResultTx(), the_limelight.txTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Y", the_limelight.returnLLResultTy(), the_limelight.tyTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Area", the_limelight.returnLLResultTa(), the_limelight.taTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Tag ID", the_limelight.returnTagId(), the_limelight.tagIdTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.

        if (the_limelight.noLimeTele()){ //same thing as the ones above but we can't use the function because the function only displays doubles as data.
            telemetry.addData("April Tag?", "No Tag Detected");
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

    @Override
    public void stop() {
        positionHold.stopHolding();
        the_limelight.limelight_stop();
        shooterToggle = false;
        indexerRunning = false;
        servos.shooterthingy(false, SHOOTER_TARGET_VELOCITY);
        servos.indexerthingy(false, indexerSpeed);
        servos.servos_move(false, intakeSpeed);
        RobotPanels.stopped();
    }

}
