package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMU_setup {

    private IMU imu;

    public void init (HardwareMap hwMap) {
        try {
            //initlizing imu
            imu = hwMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    // this is just what way the control hub is facing to know what angle its at
            );

            imu.initialize(new IMU.Parameters(RevOrientation));
        } catch (Exception e) {
            imu = null;
            // e contans the infromation of what when wrong so if that info is bad then it stopes the IMU code and moves on without errors
        }
    }

    public double getHeading(){
        if (imu == null) return 0.0;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // saying if imu is nothing then return it as zero and it wont display an error and crash the whole robot
        // this secotion is for if the getHeading() code does not work
    }

    public YawPitchRollAngles imu_orientation() {
        if (imu == null) return new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0);
        return imu.getRobotYawPitchRollAngles();
        // also if the imu id not working it creates "fake code" so the bot doesnt crash and stays happy
        // this is for if the imu_orentation() doesnt work
    }

    public void resetYaw(){
        if (imu != null) {
            imu.resetYaw();
        }
    }
}
