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
        //Init for left side
        left_wheel = hwMap.get(DcMotorEx.class,"left_wheel");
        left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Using power for troubleshooting
        tprLeftWheel = left_wheel.getMotorType().getTicksPerRev();

        //Init for right side
        right_wheel = hwMap.get(DcMotorEx.class,"right_wheel");
        right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Using power for troubleshooting
        tprRightWheel = right_wheel.getMotorType().getTicksPerRev();

        //Init intake motor
        intake_wheels = hwMap.get(DcMotor.class,"intake_wheels");
        intake_wheels.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setLeftWheelSpeed(double power){
        left_wheel.setPower(power); 
    }

    public void setRightWheelSpeed(double power){
        right_wheel.setPower(power);
    }

    public double getLeftWheelRevs(){
        return left_wheel.getCurrentPosition() / tprLeftWheel;
    }

    public double getRightWheelRevs(){
        return right_wheel.getCurrentPosition() / tprRightWheel;
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
