package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

/**
 * AprilTagAutoAlignmentMecanumDrivetrain's controller, connected to Mimir TeleOp.
 * Mimir supplies its drive inputs/runtime; the Limelight adapter supplies tag bearing.
 */
public class AutoAimTag {
    // Robot connection: rear-facing, sideways-mounted Limelight on pipeline 8.
    private static final int PIPELINE = LIMELIGHT_setup.APRILTAG_PIPELINE;
    private static final long MAX_RESULT_AGE_MS = 250;
    private Limelight3A limelight;
    private DoubleSupplier runtime;
    private LLResultTypes.FiducialResult target;
    private double targetBearing;
    private boolean alignRequested;
    private String status = "Not initialized";

    // --------------------- PD controller ----------------------
    double kP = 0.019;
    double error = 0;
    double lastError = 0;
    double goalX = 0; // offset here
    double angleTolerance = 0.4;
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    // Mimir supplies forward/strafe and applies the returned rotate value to mec_wheels.
    double rotate;

    // -------------------- controller based PD tuning -----------------------

    double[] stepSizes = {0.1, 0.001, 0.0001};
    int stepIndex = 1;

    public void init(HardwareMap hardwareMap, DoubleSupplier runtime) {
        this.runtime = runtime;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.pipelineSwitch(PIPELINE);
            limelight.start();
            status = "Initialized";
        } catch (RuntimeException e) {
            limelight = null;
            status = "Limelight unavailable: " + e.getMessage();
        }
    }

    /** Mimir resets its OpMode runtime immediately before this call. */
    public void start() {
        curTime = getRuntime();
    }

    private double getRuntime() {
        return runtime.getAsDouble();
    }

    public double getTurn(boolean redAlliance, boolean allianceKnown,
                          double manualTurn, Gamepad gamepad1) {
        // Mimir's existing stick mapping provides the manual rotation input.
        rotate = manualTurn;

        // Robot connection: replace AprilTagWebcam/id20 with the alliance Limelight tag.
        target = readAllianceTag(redAlliance, allianceKnown);
        // The existing robot used TY for horizontal aim because the camera is sideways.
        // Negate it to supply the original controller's forward-webcam bearing convention.
        targetBearing = target == null ? Double.NaN : -target.getTargetYDegrees();
        alignRequested = gamepad1.left_trigger > 0.3;

        // --------------- auto align rotation logic ----------------

        if (gamepad1.left_trigger > 0.3) {
            if (target != null) {
                error = goalX - targetBearing; // Limelight adapter replaces id20.ftcPose.bearing

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        return rotate;
    }

    /**
     * Original tuning buttons. Mimir connects gamepad2 here so B still indexes on gamepad1.
     */
    public void tune(Gamepad gamepad1) {
        // update P and D on the fly
        // 'B' button cycles through the different step sizes for tuning precision.
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        // D-pad left/right adjusts the P gain.
        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        // D-pad up/down adjusts the D gain.
        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }
    }

    // Hardware adapter only: BioBuzz IDs already used by this robot's autonomous code.
    private LLResultTypes.FiducialResult readAllianceTag(boolean redAlliance,
                                                       boolean allianceKnown) {
        if (limelight == null) return null;
        if (!allianceKnown) {
            status = "Select alliance in INIT";
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !limelight.isRunning() || !limelight.isConnected()
                || result.getStaleness() > MAX_RESULT_AGE_MS
                || result.getPipelineIndex() != PIPELINE || !result.isValid()) {
            status = "Waiting for fresh pipeline " + PIPELINE + " tag";
            return null;
        }
        int firstId = redAlliance ? 30 : 38;
        int lastId = redAlliance ? 37 : 45;
        LLResultTypes.FiducialResult selected = null;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id >= firstId && id <= lastId && Double.isFinite(tag.getTargetYDegrees())
                    && (selected == null || Math.abs(tag.getTargetYDegrees())
                    < Math.abs(selected.getTargetYDegrees()))) {
                selected = tag;
            }
        }
        status = selected == null ? "No alliance BioBuzz tag" : "Alliance tag detected";
        return selected;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (target != null) {
            if (alignRequested) {
                telemetry.addLine("AUTO ALIGN");
            }
            // Limelight's ID/bearing replace the webcam's detection telemetry.
            telemetry.addData("Aim tag", target.getFiducialId());
            telemetry.addData("Tag bearing (degrees)", targetBearing);
            telemetry.addData("Error", error);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addData("Aim camera", status);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (Gamepad 2 D-Pad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (Gamepad 2 D-Pad U/D)", kD);
        telemetry.addData("Step Size", "%.4f (Gamepad 2 B Button)", stepSizes[stepIndex]);
    }

    public void stop() {
        if (limelight != null) limelight.stop();
        target = null;
        status = "Stopped";
    }
}
