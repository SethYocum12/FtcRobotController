package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.IMU_setup;
import org.firstinspires.ftc.teamcode.mechanisms.tank_limelight;
import org.firstinspires.ftc.teamcode.mechanisms.tank_motors;
import org.firstinspires.ftc.teamcode.mechanisms.tank_servos;

@TeleOp
public class Mimir_2026_Tank extends OpMode {
    IMU_setup the_imu = new IMU_setup();
    tank_motors motors = new tank_motors();
    tank_servos servos = new tank_servos();

    tank_limelight the_limelight = new tank_limelight();


    //motors
    double left_wheel_button;
    double right_wheel_button;
    boolean intake_button;

    // limelight/april tag stuff
    private Limelight3A limelight;
    double distance;
    boolean intake_toggle = false;
    boolean last_intake_button = false;

    //power
    double drivePowerMultiplier = 0.8; // 80% max speed (setPower range is 0 to 1)
    double intakeMotorSpeed = -1;
    double intakeServoSpeed = 1;



    @Override
    public void init(){
        motors.init(hardwareMap); // initialization motors
        servos.init(hardwareMap); // Added missing initialization
        the_imu.init(hardwareMap); // initialization of IMU
        the_limelight.init(hardwareMap); //initialization of limelight


        limelight = hardwareMap.get(Limelight3A.class,"limelight");// finding april tag hw map
        limelight.pipelineSwitch(8);// finding pipeline

    }

    @Override
    public void start() {
        // this public void is used for apriltag
        the_limelight.limelight_start();

    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = the_imu.imu_orientation();// orentation for imu
        limelight.updateRobotOrientation(orientation.getYaw());// updating limelight
        LLResult llResult = limelight.getLatestResult();// pulls data from limelight
        if (llResult != null && llResult.isValid()) { // if the result is a thing and is valid do:
            distance = getDistanceFromTag((llResult.getTa())); //gets the distances from the tag
            telemetry.addData("distance", distance);
            // Check for the specific AprilTag ID. fidual is fancy word for april tag
            if (!llResult.getFiducialResults().isEmpty()) { //if the tag is not empty, return tag ID
                telemetry.addData("Tag ID", llResult.getFiducialResults().get(0).getFiducialId());
            }

            telemetry.addData("Target x", llResult.getTx()); 
            telemetry.addData("Target y", llResult.getTy()); 
            telemetry.addData("Target Area", llResult.getTa());
        } else {
            telemetry.addData("Limelight", "No AprilTag Detected");
        }

        if (intake_button && last_intake_button != intake_button){
            intake_toggle = !intake_toggle;
        }
        last_intake_button = intake_button;



        // Update gamepad inputs every loop
        left_wheel_button = -gamepad1.left_stick_y; 
        right_wheel_button = -gamepad1.right_stick_y;
        intake_button = gamepad1.a;

        // Drive Logic
        motors.setLeftWheelSpeed(left_wheel_button * drivePowerMultiplier);
        motors.setRightWheelSpeed(right_wheel_button * drivePowerMultiplier);
        motors.intake(intake_toggle, intakeMotorSpeed);
        servos.servos_move(intake_toggle, intakeServoSpeed);

        telemetry.addData("Left Stick Y", left_wheel_button);
        telemetry.addData("Right Stick Y", right_wheel_button);
        telemetry.addData("Left Power Sent", left_wheel_button * drivePowerMultiplier);
        telemetry.addData("Right Power Sent", right_wheel_button * drivePowerMultiplier);
        telemetry.addData("Left Wheel Revs", motors.getLeftWheelRevs());
        telemetry.addData("Right Wheel Revs", motors.getRightWheelRevs());
        telemetry.addData("Intake On?", intake_button);
        telemetry.addData("Heading", the_imu.getHeading(AngleUnit.DEGREES));
        if the_limelight.distanceTele(){
            telemetry.addData("Distance", the_limelight.returnDistance())
        }
        telemetry.update();
    }


}

