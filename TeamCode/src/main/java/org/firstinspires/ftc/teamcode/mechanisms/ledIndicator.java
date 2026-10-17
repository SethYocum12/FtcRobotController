
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ledIndicator {

    private Servo ledIndicator;

    private static final double RED = 0.300;
    private static final double GREEN = 0.500;
    private static final double READY_TOLERANCE = 30;

    public void init(HardwareMap hwmap) {
        ledIndicator = hwmap.get(Servo.class, "ledIndicator");
        ledIndicator.setPosition(RED);
    }

    public void indicate(boolean shooterState,
                         double shooterSpeed,
                         double targetSpeed) {

        boolean ready = shooterState && shooterSpeed >= targetSpeed - READY_TOLERANCE;
        ledIndicator.setPosition(ready ? GREEN : RED);
    }
}
