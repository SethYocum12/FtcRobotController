package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;
import org.firstinspires.ftc.teamcode.mechanisms.tank_motors;
import org.firstinspires.ftc.teamcode.mechanisms.servos;

@TeleOp
public class Mimir_2026_Tank extends OpMode {
    IMU_setup the_imu = new IMU_setup(); //creates a new imu class
    tank_motors motors = new tank_motors(); //creates a new motor class
    servos servos = new servos(); //creates a new servo class

    LIMELIGHT_setup the_limelight = new LIMELIGHT_setup(); // creates a new limelight class

    //A function used for telemtry that when a varible is  true, the telemetry is shown. But when it's false nothing happens.
    //Mainly used for external scripts
    public void externalTelemetry(String name, double data, boolean telemetryTrigger){
        if (telemetryTrigger){
            telemetry.addData(name, data);
        }
    }


    //motors
    double left_wheel_button;
    double right_wheel_button;
    boolean intake_button;

    // limelight/april tag stuff
    double distance;
    boolean intake_toggle = false;
    boolean last_intake_button = false;

    //power
    double drivePowerMultiplier = 0.8; // 80% max speed (setPower range is 0 to 1)
    double intakeMotorSpeed = 1; //sets the intake motor speed power
    double intakeServoSpeed = 1; //sets the intake servo speed power

    @Override
    public void init(){
        motors.init(hardwareMap); // initialization motors
        servos.init(hardwareMap); // Added missing initialization
        the_imu.init(hardwareMap); // initialization of IMU
        the_limelight.init(hardwareMap); //initialization of limelight
    }

    @Override
    public void start() {
        // this public void is used for apriltag
        the_limelight.limelight_start();

    }

    @Override
    public void loop() {
        // Update gamepad inputs every loop
        left_wheel_button = -gamepad1.left_stick_y; 
        right_wheel_button = -gamepad1.right_stick_y;
        intake_button = gamepad1.a;

        //button delay for intake toggle button
        if (intake_button && last_intake_button != intake_button){
            intake_toggle = !intake_toggle;
        }
        last_intake_button = intake_button;

        //The main part of every device
        motors.setLeftWheelSpeed(left_wheel_button * drivePowerMultiplier);
        motors.setRightWheelSpeed(right_wheel_button * drivePowerMultiplier);
        motors.intake(intake_toggle, intakeMotorSpeed);
        servos.servos_move(intake_toggle, intakeServoSpeed);
        the_limelight.main();

        telemetry.addData("Left Stick Y", left_wheel_button);
        telemetry.addData("Right Stick Y", right_wheel_button);
        telemetry.addData("Left Power Sent", (left_wheel_button * drivePowerMultiplier)); //shows how much power it's trying to do
        telemetry.addData("Right Power Sent", (right_wheel_button * drivePowerMultiplier)); //shows how much power it's trying to do
        telemetry.addData("Left Wheel Revs", motors.getLeftWheelRevs()); //shows the speed of the left side
        telemetry.addData("Right Wheel Revs", motors.getRightWheelRevs()); //shows the speed of the right side
        telemetry.addData("Intake On?", intake_toggle); //shows if the toggable intake is on
        telemetry.addData("Heading", the_imu.getHeading() * (180/Math.PI)); //shows the heading of the robot in degrees
        externalTelemetry("Distance", the_limelight.returnDistance(), the_limelight.distanceTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target X", the_limelight.returnLLResultTx(), the_limelight.txTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Y", the_limelight.returnLLResultTy(), the_limelight.tyTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Target Area", the_limelight.returnLLResultTa(), the_limelight.taTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.
        externalTelemetry("Tag ID", the_limelight.returnTagId(), the_limelight.tagIdTele()); //uses the external Telemetry function to only display data sometimes from the tank_limelight.java file.

        if (the_limelight.noLimeTele()){ //same thing as the ones above but we can't use the function because the function only displays doubles as data. 
            telemetry.addData("April Tag?", "No Tag Detected");
        }

        telemetry.update();
    }


}

