package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels;

@Autonomous(name = "AutoRight", group = "Autonomous")
public class AutoRight extends LinearOpMode {

    mec_wheels drive = new mec_wheels();
    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        waitForStart();

        try {
            if (!opModeIsActive()) return;

            drive.setFrontLeft(0.5);
            drive.setFrontRight(0.5);
            drive.setBackLeft(0.5);
            drive.setBackRight(0.5);

            if (!opModeIsActive()) return;

            sleep(250); // Drive for 0.636 seconds.
            if (!opModeIsActive()) return;

            drive.setFrontLeft(-0.5);
            drive.setFrontRight(0.5);
            drive.setBackLeft(0.5);
            drive.setBackRight(-0.5);

            sleep(1000); // Drive for 1 second.

            drive.setFrontLeft(0);
            drive.setFrontRight(0);
            drive.setBackLeft(0);
            drive.setBackRight(0);

        } finally {
            drive.setFrontLeft(0);
            drive.setFrontRight(0);
            drive.setBackLeft(0);
            drive.setBackRight(0);
        }
    }
}
