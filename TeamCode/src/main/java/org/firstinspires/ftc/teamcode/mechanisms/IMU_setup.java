package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMU_setup {

    private IMU imu;

    public void init (HardwareMap hwMap) {
        //initlizing imu
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double getHeading(){
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public YawPitchRollAngles imu_orientation() {
            return imu.getRobotYawPitchRollAngles();
    }

}
