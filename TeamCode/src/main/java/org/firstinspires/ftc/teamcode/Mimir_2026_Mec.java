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
    private final ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //init for the main timer



    public void externalTelemetry(String name, double data, boolean telemetryTrigger){
        if (telemetryTrigger){
            telemetry.addData(name, data);
        }
    }

    //movement variables
    double x_axis;

    double y_axis;
    double rotation;

    //powers
    double intakeSpeed = -1;

    double rotationMultiplier = 10;

    //timers

    //other

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
        runtime.reset();
    }

    @Override
    public void loop() {
        //setting gamepad variables
        x_axis = gamepad1.right_stick_x;
        y_axis = -gamepad1.right_stick_y;
        rotation = gamepad1.left_stick_x;
        turboTrigger = gamepad1.right_bumper;
        intakeTrigger = gamepad1.a;

        //calculating Mecanum power
        drive.intakeMode(intakeToggle, intakeSpeed);
        drive.turboMode(turboToggle);
        drive.main(x_axis, y_axis, rotation, rotationMultiplier);
        servos.servos_move(intakeToggle, intakeSpeed);
        the_limelight.main();

        // finding amount of power to display on driver hub
        telemetry.addData("Front L power", drive.returnFrontLeftPower());
        telemetry.addData("Front R power", drive.returnFrontRightPower());
        telemetry.addData("Back L power", drive.returnBackLeftPower());
        telemetry.addData("Back R power", drive.returnBackRightPower());
        telemetry.addData("Turbo Mode?" , turboToggle);
        telemetry.addData("Intake Mode?" , intakeToggle);
        telemetry.addData("Heading", the_imu.getHeading()); //shows the heading of the robot
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
