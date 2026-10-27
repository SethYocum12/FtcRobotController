package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.dashboard.RobotPanels;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Configurable
public class flyWheelTuning extends OpMode {
    public DcMotorEx flywheelmotor;
    public double highVelocity = 5700;
    double lowVelocity = 4000;

    double curTargetVelocity = highVelocity;
    public static volatile double F = 0;
    public static volatile double P = 0;
    double[] stepSizes = {10.0,1.0,0.1,0.001,0.0001};

    int stepIndex = 1;


    @Override
    public void init() {
        RobotPanels.attach(this);
        flywheelmotor = hardwareMap.get(DcMotorEx.class,"outake_wheel");
        flywheelmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("init complete");
        telemetry.update();


    }

    @Override public void loop() {
       //get all of our gamepad comanda
       //set target velcoity
       //update tleemetry

       if(gamepad1.yWasPressed()){
           if(curTargetVelocity == highVelocity){
               curTargetVelocity = lowVelocity;
           } else {
             curTargetVelocity = highVelocity;
           }
    }
    if (gamepad1.bWasPressed()){
        stepIndex = (stepIndex + 1) % stepSizes.length;
    }
    if (gamepad1.dpadLeftWasPressed()){
        F -= stepSizes[stepIndex];

    }
    if(gamepad1.dpadRightWasPressed()){
        F += stepSizes[stepIndex];
    }
    if(gamepad1.dpadDownWasPressed()){
        P += stepSizes[stepIndex];
    }
    if (gamepad1.dpadDownWasPressed()){
        P -= stepSizes[stepIndex];
    }

        //set new pdif coofinicants
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelmotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        //set velcotiy
        flywheelmotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelmotor.getVelocity();
        double error = curTargetVelocity - curVelocity;


        telemetry.addData("target velocity", curTargetVelocity);
        telemetry.addData("current velocity","%.2f",curVelocity);
        telemetry.addData("error","%.2f",error);
        telemetry.addLine("--------------------------");
        telemetry.addData("tuning p","%.4f (D-pad U/D",P);
        telemetry.addData("tuning f","%.4f (D-pad R/L",F);
        telemetry.addData("step size","%.4f (B button)",stepSizes[stepIndex]);
        RobotPanels.shooter(true, curVelocity, curTargetVelocity);
        telemetry.update();

    }

    @Override
    public void stop() {
        flywheelmotor.setPower(0);
        RobotPanels.stopped();
    }
}
