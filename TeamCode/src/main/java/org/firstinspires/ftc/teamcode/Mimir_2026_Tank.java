package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.tank_motors;
import org.firstinspires.ftc.teamcode.mechanisms.tank_servos;

@TeleOp
public class Mimir_2026_Tank extends OpMode {
    tank_motors motors = new tank_motors();
    tank_servos servos = new tank_servos();

    //motors
    double left_wheel_button;
    double right_wheel_button;
    boolean intake_button;

    //power
    double drivePowerMultiplier = 5; // Reduced from 10 to a safer starting value
    double intakeMotorSpeed = 1;
    double intakeServoSpeed = 1;

    @Override
    public void init(){
        motors.init(hardwareMap);
        servos.init(hardwareMap); // Added missing initialization
    }

    @Override
    public void loop() {
        // Update gamepad inputs every loop
        left_wheel_button = -gamepad1.left_stick_y; // Negative because forward is usually negative on stick
        right_wheel_button = -gamepad1.right_stick_y;
        intake_button = gamepad1.a;

        motors.setLeftWheelSpeed(left_wheel_button * drivePowerMultiplier);
        motors.setRightWheelSpeed(right_wheel_button * drivePowerMultiplier);
        motors.intake(intake_button, intakeMotorSpeed);
        servos.servos_move(intake_button, intakeServoSpeed);

        telemetry.addData("LEFT_JOYSTICK_Y", left_wheel_button);
        telemetry.addData("RIGHT_JOYSTICK_Y", right_wheel_button);
        telemetry.addData("LEFT_WHEEL_SPEED", motors.getLeftWheelRevs());
        telemetry.addData("RIGHT_WHEEL_SPEED", motors.getRightWheelRevs());
        telemetry.addData("INTAKE_ON?", intake_button);
        telemetry.addData("INTAKE_MOTOR_SPEED", motors.getIntakeMotorSpeed());
        telemetry.addData("INTAKE_LEFT_SERVO_SPEED", servos.getLeftServoPower());
        telemetry.addData("INTAKE_RIGHT_SERVO_SPEED", servos.getRightServoPower());

        telemetry.update();
    }
}

