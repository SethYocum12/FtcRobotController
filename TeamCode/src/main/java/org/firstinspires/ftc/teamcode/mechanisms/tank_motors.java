package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class tank_motors {

    private DcMotorEx left_wheel;
    private DcMotorEx right_wheel;
    private DcMotor intake_wheels;
    private double tprLeftWheel;
    private double tprRightWheel;

    public void init(HardwareMap hwMap){
        //Init for left side and setting motor RunMode
        left_wheel = hwMap.get(DcMotorEx.class,"left_wheel");
        left_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tprLeftWheel = left_wheel.getMotorType().getTicksPerRev();

        //Init for right side and setting motor RunMode
        right_wheel = hwMap.get(DcMotorEx.class,"right_wheel");
        right_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        tprRightWheel = right_wheel.getMotorType().getTicksPerRev();

        //Init intake motor
        intake_wheels = hwMap.get(DcMotor.class,"intake_wheels");
        intake_wheels.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setLeftWheelSpeed(double speed){
        //takes in rotation per minute
        left_wheel.setVelocity(speed * tprLeftWheel);

    }
    public void setRightWheelSpeed(double speed){
        //takes in rotation per minute
        right_wheel.setVelocity(speed * tprRightWheel);

    }

    public double getLeftWheelRevs(){
        return left_wheel.getCurrentPosition() / tprLeftWheel; // normalizing ticks to rev.
    }

    public double getRightWheelRevs(){
        return right_wheel.getCurrentPosition() / tprRightWheel; // normalizing ticks to rev.
    }

    public void intake(boolean on_off, double intake_speed){
        if (on_off) {
            intake_wheels.setPower(intake_speed);
        } else {
            intake_wheels.setPower(0);
        }

    }
    public double getIntakeMotorSpeed(){
        return intake_wheels.getPower();
    }
}

