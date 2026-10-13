
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ledIndicator {

    private Servo ledIndicator;

    private long lastToggleTime = 0;
    private boolean ledState = false;

    private static final long BLINK_INTERVAL = 750;

    public void init(HardwareMap hwmap) {
        ledIndicator = hwmap.get(Servo.class, "ledIndicator");
        ledOff();
    }

    public void ledOn() {
        ledIndicator.setPosition(1.0);
        ledState = true;
    }

    public void ledOff() {
        ledIndicator.setPosition(0.0);
        ledState = false;
    }

    public void indicate(boolean shooterState,
                         double shooterSpeed,
                         double targetSpeed) {

        // Shooter off: LED off
        if (!shooterState) {
            ledOff();
            lastToggleTime = 0;
            return;
        }

        // Shooter below target speed: blink LED
        if (shooterSpeed < targetSpeed) {

            long currentTime = System.currentTimeMillis();

            if (currentTime - lastToggleTime >= BLINK_INTERVAL) {

                if (ledState) {
                    ledOff();
                } else {
                    ledOn();
                }

                lastToggleTime = currentTime;
            }

        } else {
            // Shooter at target speed: solid LED
            ledOn();
        }
    }
}
