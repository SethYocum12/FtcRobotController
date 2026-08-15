package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class servos {

    private CRServo left_servo;
    private CRServo right_servo;

    public void init(HardwareMap hwMap){
        left_servo = hwMap.get(CRServo.class,"left_servo");
        left_servo.setDirection(DcMotorSimple.Direction.REVERSE);
        right_servo = hwMap.get(CRServo.class,"right_servo");
    }

    public void servos_move(boolean activation_button, double intake_speed) {
        if (activation_button) {
            left_servo.setPower(intake_speed);
            right_servo.setPower(intake_speed);
        } else {
            left_servo.setPower(0);
            right_servo.setPower(0);
        }
    }

    public double getLeftServoPower(){
        return left_servo.getPower();
    }

    public double getRightServoPower(){
        return right_servo.getPower();
    }
}

