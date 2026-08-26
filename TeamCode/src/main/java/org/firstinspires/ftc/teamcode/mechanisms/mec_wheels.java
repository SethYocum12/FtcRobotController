package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mec_wheels {

    //Motors
    private DcMotorEx front_left;
    double tprFrontLeft;
    double frontLPower;

    private DcMotorEx front_right;
    double tprFrontRight;
    double frontRPower;

    private DcMotorEx back_left;
    double tprBackLeft;
    double backLPower;

    private DcMotorEx back_right;
    double tprBackRight;
    double backRPower;

    private DcMotor intake_wheels;

    double driveSpeed = 0.6;

    //init for everything
    public void init(HardwareMap hwMap){

        //DcMotor Init
        front_left = hwMap.get(DcMotorEx.class, "front_left");
        front_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        tprFrontLeft = front_left.getMotorType().getTicksPerRev();

        front_right = hwMap.get(DcMotorEx.class, "front_right");
        front_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        tprFrontRight = front_right.getMotorType().getTicksPerRev();

        back_left = hwMap.get(DcMotorEx.class, "back_left");
        back_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        tprBackLeft = back_left.getMotorType().getTicksPerRev();

        back_right = hwMap.get(DcMotorEx.class, "back_right");
        back_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right.setDirection(DcMotorEx.Direction.REVERSE);
        tprBackRight = back_right.getMotorType().getTicksPerRev();

        intake_wheels = hwMap.get(DcMotor.class,"intake_wheels");
        intake_wheels.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setFrontLeft(double rps){
        // -1 to 1
        front_left.setVelocity(rps * tprFrontLeft);

    }

        public void setFrontRight(double rps){
        // -1 to 1
        front_right.setVelocity(rps * tprFrontRight);

    }

    public void setBackLeft(double rps){
        // -1 to 1
        back_left.setVelocity(rps * tprBackLeft);

    }

    public void setBackRight(double rps){
        // -1 to 1
        back_right.setVelocity(rps * tprBackRight);

    }

    public double returnFrontLeftPower(){
        return front_left.getVelocity() / tprFrontLeft;
    }

    public double returnFrontRightPower(){
        return front_right.getVelocity() / tprFrontRight;
    }

    public double returnBackLeftPower(){
        return back_left.getVelocity() / tprBackLeft;
    }

    public double returnBackRightPower(){

        return back_right.getVelocity() / tprBackRight;
    }
    public void turboMode(boolean turboToggle){
        if (turboToggle){
            driveSpeed = 1;
        } else {
            driveSpeed = 0.6;
        }
    }

    public void intakeMode(boolean intakeToggle, double intakeSpeed){
        if (intakeToggle) {
            intake_wheels.setPower(intakeSpeed);
        } else {
            intake_wheels.setPower(0);
        }
    }

    public void main(double x_axis, double y_axis, double rotation, double powerMulti){
        frontLPower = (y_axis + x_axis) + rotation;
        frontRPower = (y_axis - x_axis) - rotation;
        backLPower = (y_axis - x_axis) + rotation;
        backRPower = (y_axis + x_axis) - rotation;

        // Normalize the values so none exceed +/- 1.0
        double max = Math.max(Math.abs(frontLPower), Math.abs(frontRPower));
        max = Math.max(max, Math.abs(backLPower));
        max = Math.max(max, Math.abs(backRPower));

        if (max > 1.0) {
            frontLPower /= max;
            frontRPower /= max;
            backLPower /= max;
            backRPower /= max;
        }

        // Set Final Power
        this.setFrontLeft(frontLPower * powerMulti * driveSpeed);
        this.setFrontRight(frontRPower * powerMulti * driveSpeed);
        this.setBackLeft(backLPower * powerMulti * driveSpeed);
        this.setBackRight(backRPower * powerMulti * driveSpeed);
    }


}

