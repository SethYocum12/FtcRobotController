package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels_mechanisms;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Mimir_2026_Mec extends OpMode {

    mec_wheels_mechanisms drive = new mec_wheels_mechanisms();
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public double divide (double input, double divider){
        return input / divider;
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
    double turboPower = 1.0;
    double maxPower = normalPower;

    //timers
    double buttonDelay = 300;

    double last_turbo = -buttonDelay;

    //other
    double EqualizationPower;

    //turbo
    boolean turbo_mode_button;
    boolean turbo_mode = false;

    @Override
    public void init() {
        //init motors and servos.
        drive.init(hardwareMap);
        runtime.reset();
    }

    @Override
    public void loop() {
        //setting gamepad variables
        x_axis = gamepad1.right_stick_x;
        y_axis = -gamepad1.right_stick_y;
        rotation = gamepad1.left_stick_x;
        turbo_mode_button = gamepad1.dpad_up;
        //calculating Mecanum power
        frontLPower = (y_axis + x_axis) + rotation;
        frontRPower = (y_axis - x_axis) - rotation;
        backLPower = (y_axis - x_axis) + rotation;
        backRPower = (y_axis + x_axis) - rotation;
        //Finds the equalization power by finding the max of all the powers.
        EqualizationPower = JavaUtil.maxOfList(JavaUtil.createListWith(
                Math.abs(frontLPower),
                Math.abs(frontRPower),
                Math.abs(backLPower),
                Math.abs(backRPower)));
        // Does equalization to make code not go over 1 and keep movement speed ratio between different motors & wheels.
        if (EqualizationPower > 1) {
            frontLPower = (float) divide(frontLPower, EqualizationPower);
            frontRPower = (float) divide(frontRPower, EqualizationPower);
            backLPower = (float) divide(backLPower, EqualizationPower);
            backRPower = (float) divide(backRPower, EqualizationPower);
        }
        // Set Final Power
        drive.setFrontLeft(frontLPower * maxPower);
        drive.setFrontRight(frontRPower * maxPower);
        drive.setBackLeft(backLPower * maxPower);
        drive.setBackRight(backRPower * maxPower);

        // finding amount of power to display on driver hub
        telemetry.addData("Front L power", drive.returnFrontLeftPower());
        telemetry.addData("Front R power", drive.returnFrontRightPower());
        telemetry.addData("Back L power", drive.returnBackLeftPower());
        telemetry.addData("Back R power", drive.returnBackRightPower());
        telemetry.addData("Turbo Mode?" , turbo_mode);
        telemetry.addData("Power" , maxPower);

        telemetry.update();

        if (turbo_mode_button) {
            if ((runtime.milliseconds() - last_turbo) > buttonDelay) {
                turbo_mode = !turbo_mode;
                last_turbo = runtime.milliseconds();
                if (turbo_mode) {
                    maxPower = turboPower;
                } else {
                    maxPower = normalPower;
                }
            }
        }
    }
}