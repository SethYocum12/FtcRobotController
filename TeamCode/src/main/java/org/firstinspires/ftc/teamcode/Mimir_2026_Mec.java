package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;
import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels;
import org.firstinspires.ftc.teamcode.mechanisms.servos;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Mimir_2026_Mec extends OpMode {

    mec_wheels drive = new mec_wheels(); //creates a new drive class
    IMU_setup the_imu = new IMU_setup(); //creates a new imu class
    servos servos = new servos(); //creates a new servo class
    LIMELIGHT_setup the_limelight = new LIMELIGHT_setup(); // creates a new limelight class
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
    double rotationMultiplier = 2;

    //other
    boolean resetYaw;
    //intake

    boolean intakeToggle = false;
    boolean intakeTrigger = false;
    boolean lastIntakeTrigger = false;

    //turbo

    boolean turboToggle = false;
    boolean turboTrigger = false;
    boolean lastTurboTrigger = false;

    @Override
    public void init() {
        //init motors and servos.
        drive.init(hardwareMap);
        servos.init(hardwareMap); // Added missing initialization
        the_imu.init(hardwareMap); // initialization of IMU
        the_limelight.init(hardwareMap); //initialization of limelight
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

        //calculating Mecanum power
        double botHeading = -the_imu.getHeading();
        rotX = x_axis * Math.cos(botHeading) - y_axis * Math.sin(botHeading);
        rotY = x_axis * Math.sin(botHeading) + y_axis * Math.cos(botHeading);

        drive.intakeMode(intakeToggle, intakeSpeed);
        drive.turboMode(turboToggle);
        drive.main(rotX, rotY, rotation, rotationMultiplier);
        servos.servos_move(intakeToggle, intakeSpeed);
        the_limelight.main();

        if (resetYaw) {
            the_imu.resetYaw();
        }

        // Measured wheel speeds from the motor encoders
        telemetry.addData("Front L speed (rev/s)", drive.returnFrontLeftPower());
        telemetry.addData("Front R speed (rev/s)", drive.returnFrontRightPower());
        telemetry.addData("Back L speed (rev/s)", drive.returnBackLeftPower());
        telemetry.addData("Back R speed (rev/s)", drive.returnBackRightPower());
        telemetry.addData("Gamepad1 A" , intakeTrigger);
        telemetry.addData("Turbo Mode?" , turboToggle);
        telemetry.addData("Intake Mode?" , intakeToggle);
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

}
