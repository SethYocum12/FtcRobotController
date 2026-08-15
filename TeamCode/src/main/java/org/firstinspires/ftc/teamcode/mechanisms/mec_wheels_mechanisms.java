package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class mec_wheels_mechanisms {

    //Motors
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    double EqualizationPower;

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

    public void main(double x_axis, double y_axis, double rotation, double maxPower, double frontLPower, double frontRPower, double backLPower, double backRPower){
        frontLPower = (y_axis + x_axis) + rotation;
        frontRPower = (y_axis - x_axis) - rotation;
        backLPower = (y_axis - x_axis) + rotation;
        backRPower = (y_axis + x_axis) - rotation;
        //Finds the equalization power by finding the max of all the powers.
        this.EqualizationPower = JavaUtil.maxOfList(JavaUtil.createListWith(
                Math.abs(frontLPower),
                Math.abs(frontRPower),
                Math.abs(backLPower),
                Math.abs(backRPower)));
        // Does equalization to make code not go over 1 and keep movement speed ratio between different motors & wheels.
        if (this.EqualizationPower > 1) {
            frontLPower = (frontLPower / this.EqualizationPower);
            frontRPower = (frontRPower / this.EqualizationPower);
            backLPower = (backLPower / this.EqualizationPower);
            backRPower = (backRPower / this.EqualizationPower);
        }
        // Set Final Power
        this.setFrontLeft(frontLPower * maxPower);
        this.setFrontRight(frontRPower * maxPower);
        this.setBackLeft(backLPower * maxPower);
        this.setBackRight(backRPower * maxPower);
    }


}

