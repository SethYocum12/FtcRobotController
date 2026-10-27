package org.firstinspires.ftc.teamcode.dashboard;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.lights.PanelsLights;
import com.bylazar.lights.RGBIndicator;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.LIMELIGHT_setup;

import java.util.ArrayDeque;
import java.util.Deque;

/** Dashboard output only: this class never commands motors or replaces Driver Station inputs. */
@Configurable
public final class RobotPanels {
    public static volatile boolean SHOW_TRAIL = true;
    public static volatile double ROBOT_RADIUS_INCHES = 9.0;
    // TeleOp localization starts relative to INIT unless these are set to a known field pose.
    public static volatile double TELEOP_START_X = 0;
    public static volatile double TELEOP_START_Y = 0;
    public static volatile double TELEOP_START_HEADING_DEGREES = 0;

    private static final int TRAIL_POINTS = 150;
    private static final int PATH_STEPS = 60;
    private static final Deque<Pose> trail = new ArrayDeque<>();
    private static final RGBIndicator shooterLight = new RGBIndicator("Shooter ready");
    private static final RGBIndicator visionLight = new RGBIndicator("Limelight target");
    @IgnoreConfigurable private static long lastFieldNanos;
    @IgnoreConfigurable private static Path[] plannedPaths = new Path[0];
    @IgnoreConfigurable private static Pose latestPose;
    @IgnoreConfigurable private static long lastTickNanos;

    private RobotPanels() { }

    /** Call at the start of init/runOpMode, after the SDK has reset its telemetry. */
    public static void attach(OpMode opMode) {
        if (opMode.telemetry instanceof JoinedTelemetry) return;
        Telemetry driverStation = opMode.telemetry;
        driverStation.setMsTransmissionInterval(100);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().clearAll();
        PanelsTelemetry.INSTANCE.getTelemetry().setUpdateInterval(100);
        opMode.telemetry = new JoinedTelemetry(driverStation,
                PanelsTelemetry.INSTANCE.getFtcTelemetry());
        // Runs on the OpMode thread when DS telemetry is transmitted, before Panels.update().
        driverStation.addAction(() -> {
            TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
            panels.addData("OpMode", opMode.getClass().getSimpleName());
            if (latestPose != null) {
                panels.addData("Pose X (inches)", latestPose.x());
                panels.addData("Pose Y (inches)", latestPose.y());
                panels.addData("Pose heading (degrees)", Math.toDegrees(latestPose.heading()));
            }
            gamepad(panels, "Driver 1", opMode.gamepad1);
            gamepad(panels, "Driver 2", opMode.gamepad2);
        });
        trail.clear();
        plannedPaths = new Path[0];
        latestPose = null;
        lastTickNanos = 0;
        lastFieldNanos = 0;
        FieldManager field = PanelsField.INSTANCE.getField();
        field.getCanvas().getItems().clear();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        field.update();
        shooterLight.reset();
        visionLight.reset();
        PanelsLights.INSTANCE.getLights().initLights(shooterLight, visionLight);
        PanelsLights.INSTANCE.getLights().update();
    }

    private static void gamepad(TelemetryManager panels, String prefix, Gamepad g) {
        panels.addData(prefix + " left stick X", g.left_stick_x);
        panels.addData(prefix + " left stick Y", g.left_stick_y);
        panels.addData(prefix + " right stick X", g.right_stick_x);
        panels.addData(prefix + " right stick Y", g.right_stick_y);
        panels.addData(prefix + " left trigger", g.left_trigger);
        panels.addData(prefix + " right trigger", g.right_trigger);
        panels.addData(prefix + " A", g.a ? 1 : 0);
        panels.addData(prefix + " B", g.b ? 1 : 0);
        panels.addData(prefix + " X", g.x ? 1 : 0);
        panels.addData(prefix + " Y", g.y ? 1 : 0);
        panels.addData(prefix + " left bumper", g.left_bumper ? 1 : 0);
        panels.addData(prefix + " right bumper", g.right_bumper ? 1 : 0);
        panels.addData(prefix + " dpad up", g.dpad_up ? 1 : 0);
        panels.addData(prefix + " dpad down", g.dpad_down ? 1 : 0);
        panels.addData(prefix + " dpad left", g.dpad_left ? 1 : 0);
        panels.addData(prefix + " dpad right", g.dpad_right ? 1 : 0);
        panels.addData(prefix + " left stick button", g.left_stick_button ? 1 : 0);
        panels.addData(prefix + " right stick button", g.right_stick_button ? 1 : 0);
        panels.addData(prefix + " start", g.start ? 1 : 0);
        panels.addData(prefix + " back", g.back ? 1 : 0);
        panels.addData(prefix + " guide", g.guide ? 1 : 0);
    }

    /** Draws a full routine during INIT, then highlights the active path during RUN. */
    public static void preview(Path... paths) {
        plannedPaths = paths;
    }

    public static void follower(Follower follower) {
        pose(follower.pose(), follower.currentPath(), plannedPaths);
    }

    public static void pose(Pose pose) {
        pose(pose, null, new Path[0]);
    }

    private static void pose(Pose pose, Path activePath, Path[] preview) {
        latestPose = pose;
        long now = System.nanoTime();
        if (now - lastFieldNanos < 100_000_000L) return;
        if (!Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) return;
        lastFieldNanos = now;
        FieldManager field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        field.clearFill();
        field.setOutline("#64748b", 0.7);
        for (Path path : preview) drawPath(field, path);
        field.setOutline("#facc15", 1.0);
        if (activePath != null) drawPath(field, activePath);

        if (SHOW_TRAIL) {
            trail.addLast(pose);
            while (trail.size() > TRAIL_POINTS) trail.removeFirst();
            field.setOutline("#22d3ee", 0.6);
            Pose previous = null;
            for (Pose point : trail) {
                if (previous != null) {
                    field.moveCursor(previous.x(), previous.y());
                    field.line(point.x(), point.y());
                }
                previous = point;
            }
        } else {
            trail.clear();
        }
        double radius = Double.isFinite(ROBOT_RADIUS_INCHES)
                ? Math.max(1, Math.min(18, ROBOT_RADIUS_INCHES)) : 9;
        field.setStyle("#22d3ee33", "#22d3ee", 1.0);
        field.moveCursor(pose.x(), pose.y());
        field.circle(radius);
        field.moveCursor(pose.x(), pose.y());
        field.line(pose.x() + radius * Math.cos(pose.heading()),
                pose.y() + radius * Math.sin(pose.heading()));
        field.update();
    }

    /** For existing tuning loops that do not otherwise call telemetry.update(). */
    public static void tick(OpMode opMode) {
        long now = System.nanoTime();
        if (now - lastTickNanos < 100_000_000L) return;
        lastTickNanos = now;
        opMode.telemetry.update();
    }

    private static void drawPath(FieldManager field, Path path) {
        Pose start = path.get(0);
        field.moveCursor(start.x(), start.y());
        for (int i = 1; i <= PATH_STEPS; i++) {
            Pose next = path.get(i / (double) PATH_STEPS);
            field.line(next.x(), next.y());
            field.moveCursor(next.x(), next.y());
        }
    }

    public static void vision(LIMELIGHT_setup vision) {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        panels.addData("Limelight status", vision.getStatus());
        panels.addData("Limelight connected", vision.isConnected());
        panels.addData("Limelight pipeline", vision.getPipelineIndex());
        panels.addData("Limelight pipeline type", vision.getPipelineType());
        panels.addData("Limelight frame age (ms)", vision.getResultAgeMs());
        panels.addData("Limelight tag IDs", vision.getTagIds());
        panels.addData("Limelight fresh target", vision.tagIdTele() ? 1 : 0);
        // Omit invalid measurements instead of graphing missing detections as zero.
        if (vision.tagIdTele()) {
            panels.addData("Limelight TX (degrees)", vision.returnLLResultTx());
            panels.addData("Limelight TY (degrees)", vision.returnLLResultTy());
            panels.addData("Limelight target area (%)", vision.returnLLResultTa());
        }
        if (vision.distanceTele()) {
            panels.addData("Limelight empirical range estimate", vision.returnDistance());
        }
        visionLight.update(vision.tagIdTele() ? RGBIndicator.Companion.getGREEN()
                : vision.isConnected() ? RGBIndicator.Companion.getYELLOW()
                : RGBIndicator.Companion.getRED());
        PanelsLights.INSTANCE.getLights().update();
    }

    public static void shooter(boolean enabled, double actual, double target) {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        panels.addData("Shooter enabled", enabled ? 1 : 0);
        panels.addData("Shooter measured (ticks/s)", actual);
        panels.addData("Shooter requested (ticks/s)", enabled ? target : 0);
        panels.addData("Shooter tracking error (ticks/s)", (enabled ? target : 0) - actual);
        shooterLight.update(!enabled ? RGBIndicator.Companion.getOFF()
                : actual >= target - 30 ? RGBIndicator.Companion.getGREEN()
                : RGBIndicator.Companion.getRED());
        PanelsLights.INSTANCE.getLights().update();
    }

    public static void stopped() {
        shooterLight.reset();
        visionLight.reset();
        PanelsLights.INSTANCE.getLights().setLastUpdate(0);
        PanelsLights.INSTANCE.getLights().update();
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        panels.getLines().clear();
        panels.addLine("OpMode stopped");
        panels.setLastUpdate(0);
        panels.update();
    }
}
