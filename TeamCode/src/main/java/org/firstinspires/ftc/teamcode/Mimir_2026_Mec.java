package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.mec_wheels;
import org.firstinspires.ftc.teamcode.mechanisms.servos;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Mimir_2026_Mec extends OpMode {

    mec_wheels drive = new mec_wheels(); //creates a new drive class
    servos servos = new servos(); //creates a new servo class

    private final ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //init for the main timer

    public void externalTelemetry(String name, double data, boolean telemetryTrigger){
        if (telemetryTrigger){
            telemetry.addData(name, data);
        }
    }

    //movement variables
    double x_axis;

    double y_axis;
    double rotation;

    //powers
    double intakeSpeed = 1;

    double rotationMultiplier = 2;

    //timers

    //other

    //intake

    boolean intakeToggle = false;
    boolean intakeTrigger = false;
    boolean lastIntakeTrigger = false;

    //turbo

    boolean turboToggle = false;
    boolean turboTrigger = false;
    boolean lastTurboTrigger = false;

    @Override
    public void init() {
        //init motors and servos.
        drive.init(hardwareMap);
        servos.init(hardwareMap); // Added missing initialization
        runtime.reset();
    }

    @Override
    public void loop() {
        //setting gamepad variables
        x_axis = gamepad1.right_stick_x;
        y_axis = -gamepad1.right_stick_y;
        rotation = gamepad1.left_stick_x;
        turboTrigger = gamepad1.right_bumper;
        intakeTrigger = gamepad1.a;

        //calculating Mecanum power
        drive.intakeMode(intakeToggle, intakeSpeed);
        drive.turboMode(turboToggle);
        drive.main(x_axis, y_axis, rotation, rotationMultiplier);
        servos.servos_move(intakeToggle, intakeSpeed);

        // finding amount of power to display on driver hub
        telemetry.addData("Front L power", drive.returnFrontLeftPower());
        telemetry.addData("Front R power", drive.returnFrontRightPower());
        telemetry.addData("Back L power", drive.returnBackLeftPower());
        telemetry.addData("Back R power", drive.returnBackRightPower());
        telemetry.addData("Turbo Mode?" , turboToggle);
        telemetry.addData("Intake Mode?" , intakeToggle);

        telemetry.update();

        //turbo toggle
        if (turboTrigger && !lastTurboTrigger) {
            turboToggle = !turboToggle;
        }
        lastTurboTrigger = turboTrigger;

        //intake toggle
        if (intakeTrigger && !lastIntakeTrigger) {
            intakeToggle = !intakeToggle;
        }
        lastIntakeTrigger = intakeTrigger;
    }
}
