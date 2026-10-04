package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.os.Binder;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class servos {

    private CRServo left_servo;
    private CRServo right_servo;

    private CRServo indexer_wheel;
    private DcMotor intake_wheels;
    private DcMotorEx outake_wheel;
    boolean lastIndexerToggle = false;

    public void init(HardwareMap hwMap){
        left_servo = hwMap.get(CRServo.class,"left_servo");
        left_servo.setDirection(DcMotorSimple.Direction.REVERSE);
        right_servo = hwMap.get(CRServo.class,"right_servo");
        right_servo.setDirection(DcMotorSimple.Direction.FORWARD);
        indexer_wheel = hwMap.get(CRServo.class, "indexer_wheel");
        indexer_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_wheels = hwMap.get(DcMotor.class,"intake_wheels");
        intake_wheels.setDirection(DcMotorSimple.Direction.REVERSE);
        outake_wheel = hwMap.get(DcMotorEx.class, "outake_wheel");
        outake_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outake_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        outake_wheel.setVelocityPIDFCoefficients(40.0, 0.0, 0.0, 12.5);
    }

    public double getShooterRPM() {
        return Math.abs(outake_wheel.getVelocity()) * 60.0 / 28.0;
    }

    public double getShooterVelocity() {
        return Math.abs(outake_wheel.getVelocity()); // Encoder ticks per second
    }

    public void servos_move(boolean activation_button, double intake_speed) {
        if (activation_button) {
            left_servo.setPower(intake_speed);
            right_servo.setPower(intake_speed);
            intake_wheels.setPower(intake_speed);
        } else {
            left_servo.setPower(0);
            right_servo.setPower(0);
            intake_wheels.setPower(0);
        }
    }

    public void indexerthingy( boolean indexerToggle,double intake_speed) {
     if(indexerToggle == true){
         indexer_wheel.setPower(intake_speed);
     } else {
         indexer_wheel.setPower(0);
     }
    }

    public void indexerthingyBack( boolean indexerToggle,double intake_speed) {
        if(indexerToggle == true){
            indexer_wheel.setPower(intake_speed);
        } else {
            indexer_wheel.setPower(0);
        }
    }
    public void shooterthingy(boolean shooterToggle, double targetVelocity) {
        if(shooterToggle == true) {
            // Preserve the direction previously used with intakeSpeed = -1.
            outake_wheel.setVelocity(-targetVelocity);
        }
        else {
            outake_wheel.setPower(0);
        }
    }




    public double getLeftServoPower(){
        return left_servo.getPower();
    }

    public double getRightServoPower(){
        return right_servo.getPower();
    }
}

