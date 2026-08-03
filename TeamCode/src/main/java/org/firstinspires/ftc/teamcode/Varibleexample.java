package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Varibleexample extends OpMode {
    @Override
    public void init() {
      double motorSpeed = 0.75;
      boolean power = true;

      telemetry.addData("mortor vroom vroom",motorSpeed);
      telemetry.addData("power status",power);
    }

    @Override
    public void loop() {

    }
}


