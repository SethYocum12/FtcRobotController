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

    public double divide (double input, double divider){
        return input / divider;
    }

    public void toggleButton(boolean buttonTrigger, boolean lastButtonTrigger, boolean buttonToggle){
        if (buttonTrigger && lastButtonTrigger != buttonTrigger){
            buttonToggle = !buttonToggle;
        }
        lastButtonTrigger = buttonTrigger;
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

    double last_turbo = -buttonDelay;

    //other
    double EqualizationPower;

    //turbo
    boolean turboToggle = false;
    double turboPower = 1.0;
    boolean turboTrigger = gamepad1.dpad_up;
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
        //calculating Mecanum power
        drive.main(x_axis, y_axis, rotation, maxPower, frontLPower, frontRPower, backLPower, backRPower);
        // finding amount of power to display on driver hub
        telemetry.addData("Front L power", drive.returnFrontLeftPower());
        telemetry.addData("Front R power", drive.returnFrontRightPower());
        telemetry.addData("Back L power", drive.returnBackLeftPower());
        telemetry.addData("Back R power", drive.returnBackRightPower());
        telemetry.addData("Turbo Mode?" , turboToggle);
        telemetry.addData("Power" , maxPower);

        telemetry.update();

        toggleButton(turboTrigger, lastTurboTrigger, turboToggle);
    }
}