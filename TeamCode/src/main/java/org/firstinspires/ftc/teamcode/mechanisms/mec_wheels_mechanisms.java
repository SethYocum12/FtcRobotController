package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mec_wheels_mechanisms {

    //Motors
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    //init for everything
    public void init(HardwareMap hwMap){

        //DcMotor Init
        front_left = hwMap.get(DcMotor.class, "front_left");
        front_right = hwMap.get(DcMotor.class, "front_right");
        back_left = hwMap.get(DcMotor.class, "back_left");
        back_right = hwMap.get(DcMotor.class, "back_right");

        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setFrontLeft(double power){
        // -1 to 1
        front_left.setPower(power);

    }

    public void setFrontRight(double power){
        // -1 to 1
        front_right.setPower(power);

    }

    public void setBackLeft(double power){
        // -1 to 1
        back_left.setPower(power);

    }

    public void setBackRight(double power){
        // -1 to 1
        back_right.setPower(power);

    }

    public double returnFrontLeftPower(){
        return front_left.getPower();
    }

    public double returnFrontRightPower(){
        return front_right.getPower();
    }

    public double returnBackLeftPower(){
        return back_left.getPower();
    }

    public double returnBackRightPower(){
        return back_right.getPower();
    }


    public double returnJoystick (double axis){

        return axis;
    }


}

