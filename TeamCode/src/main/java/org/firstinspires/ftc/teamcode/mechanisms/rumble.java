package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

/** Standalone controller rumble; no robot hardware is initialized. */
@TeleOp(name = "Rumble", group = "Practice")
public class rumble extends OpMode {
    private boolean enabled;
    private boolean lastX;

    @Override
    public void init() {
        enabled = false;
        gamepad1.stopRumble();
        telemetry.addLine("START, then press X to toggle controller rumble.");
        telemetry.update();
    }

    @Override
    public void start() {
        lastX = gamepad1.x;
    }

    @Override
    public void loop() {
        if (gamepad1.x && !lastX) {
            enabled = !enabled;
            if (enabled) {
                gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
                gamepad1.stopRumble();
            }
        }
        lastX = gamepad1.x;

        telemetry.addData("Rumble", enabled ? "ON" : "OFF");
        telemetry.addLine("Press X again to toggle. STOP also ends rumble.");
        telemetry.update();
    }

    @Override
    public void stop() {
        enabled = false;
        gamepad1.stopRumble();
    }
}
