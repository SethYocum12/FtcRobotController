package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;
import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels_mechanisms;
import org.firstinspires.ftc.teamcode.mechanisms.servos;
import org.firstinspires.ftc.teamcode.mechanisms.tank_motors;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Mimir_2026_Mec extends OpMode {

    mec_wheels_mechanisms drive = new mec_wheels_mechanisms();
    IMU_setup the_imu = new IMU_setup(); //creates a new imu class
    servos servos = new servos(); //creates a new servo class
    LIMELIGHT_setup the_limelight = new LIMELIGHT_setup(); // creates a new limelight class
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void toggleButton(boolean buttonTrigger, boolean lastButtonTrigger, boolean buttonToggle){
        if (buttonTrigger && lastButtonTrigger != buttonTrigger){
            buttonToggle = !buttonToggle;
        }
        lastButtonTrigger = buttonTrigger;
    }

    public void externalTelemetry(String name, double data, boolean telemetryTrigger){
        if (telemetryTrigger){
            telemetry.addData(name, data);
        }
    }

    //movement variables
    double x_axis;

    double x_axis_button = gamepad1.right_stick_x;
    double y_axis;
    double rotation;

    //motor powers
    double frontLPower;
    double frontRPower;
    double backLPower;
    double backRPower;

    //powers

    double normalPower = 0.65;

    double maxPower = normalPower;

    //timers
    double buttonDelay = 300;

    //other

    //intake
    boolean intakeButton = gamepad1.a;
    boolean intakeToggle = false;
    boolean intakeTrigger = false;
    boolean lastIntakeTrigger;

    //turbo
    boolean turboButton = gamepad1.right_bumper;
    boolean turboToggle = false;
    boolean turboTrigger = false;
    boolean lastTurboTrigger;

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
        turboTrigger = turboButton;
        intakeTrigger = intakeButton;
        //calculating Mecanum power
        drive.main(x_axis, y_axis, rotation, maxPower, frontLPower, frontRPower, backLPower, backRPower);
        the_limelight.main();
        // finding amount of power to display on driver hub
        telemetry.addData("Front L power", drive.returnFrontLeftPower());
        telemetry.addData("Front R power", drive.returnFrontRightPower());
        telemetry.addData("Back L power", drive.returnBackLeftPower());
        telemetry.addData("Back R power", drive.returnBackRightPower());
        telemetry.addData("Turbo Mode?" , turboToggle);
        telemetry.addData("Power" , maxPower);
        externalTelemetry("Target X", the_limelight.returnLLResultTx(), the_limelight.txTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Y", the_limelight.returnLLResultTy(), the_limelight.tyTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Area", the_limelight.returnLLResultTa(), the_limelight.taTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Tag ID", the_limelight.returnTagId(), the_limelight.tagIdTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.

        if (the_limelight.noLimeTele()){ //same thing as the ones above but we can't use the function because the function only displays doubles as data.
            telemetry.addData("April Tag?", "No Tag Detected");
        }

        telemetry.update();

        toggleButton(turboTrigger, lastTurboTrigger, turboToggle);
        toggleButton(intakeTrigger, lastIntakeTrigger, intakeToggle);
    }
}