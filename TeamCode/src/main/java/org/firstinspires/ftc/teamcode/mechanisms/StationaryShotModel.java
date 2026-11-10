package org.firstinspires.ftc.teamcode.mechanisms;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

import static org.firstinspires.ftc.teamcode.mechanisms.ShotTrajectoryMath.*;

/** Connects a verified fixed-speed trajectory to live per-tag 3D camera observations. */
public final class StationaryShotModel {
    private final Properties data = new Properties();
    private File file;
    private boolean calibrated;
    private String calibrationStatus = "Math needs field calibration";
    private String status = calibrationStatus;
    private Rotation cameraRotation;
    private Vec3 cameraOffset, release, tagBottom, tagRight, tagUp, launch;
    private double targetVelocity, minDistance, maxDistance, radius, margin;
    private Prediction prediction;

    /** Writes an unverified template on first use; never invents fitted launch parameters. */
    public void load(File file) {
        this.file = file;
        calibrated = false;
        prediction = null;
        defaults();
        try {
            if (file.isFile()) {
                try (FileInputStream input = new FileInputStream(file)) { data.load(input); }
            } else {
                store();
            }
            if (!"1".equals(data.getProperty("version"))
                    || !StationaryShotCue.FIELD_GEOMETRY.equals(data.getProperty("fieldGeometry"))) {
                throw new IOException("Model format or field geometry changed");
            }
            if (!Boolean.parseBoolean(data.getProperty("camera.verified"))
                    || !Boolean.parseBoolean(data.getProperty("opening.verified"))) {
                calibrationStatus = "Prediction needs practice recordings";
                status = calibrationStatus;
                return;
            }
            if (!Boolean.parseBoolean(data.getProperty("trajectory.validated"))) {
                calibrationStatus = "Math: fit trajectory and check LEFT/RIGHT results";
                status = calibrationStatus;
                return;
            }
            cameraRotation = Rotation.yawPitchRoll(Math.toRadians(number("camera.yawDegrees")),
                    Math.toRadians(number("camera.pitchDegrees")), Math.toRadians(number("camera.rollDegrees")));
            cameraOffset = vector("camera.position");
            release = vector("muzzle.position");
            tagBottom = vector("opening.bottomAtClusterCenter");
            tagRight = vector("opening.right");
            tagUp = vector("opening.up");
            targetVelocity = number("trajectory.targetTicksPerSecond");
            double horizontal = number("trajectory.horizontalInchesPerSecond");
            double vertical = number("trajectory.verticalInchesPerSecond");
            double yaw = Math.toRadians(number("trajectory.robotYawDegrees"));
            launch = new Vec3(horizontal*Math.cos(yaw), horizontal*Math.sin(yaw), vertical);
            minDistance = number("trajectory.minimumDistanceInches");
            maxDistance = number("trajectory.maximumDistanceInches");
            radius = number("trajectory.ballRadiusInches");
            margin = number("trajectory.errorMarginInches");
            double residual = number("trajectory.maximumFitErrorInches");
            int trials = Integer.parseInt(data.getProperty("validation.shots"));
            int successes = Integer.parseInt(data.getProperty("validation.scored"));
            if (!cameraRotation.valid() || !cameraOffset.finite() || !release.finite()
                    || !new Opening(tagBottom, tagRight, tagUp, new double[][]{{0,0},{1,0},{0,1}}).valid()
                    || !launch.finite() || horizontal <= 0 || vertical <= 0 || targetVelocity <= 0
                    || minDistance <= 0 || maxDistance <= minDistance || radius < 1.4
                    || residual < 0 || margin < 1.0 + residual
                    || trials < 20 || successes < 0 || successes > trials || successes < 0.9*trials) {
                throw new IOException("Incomplete or invalid field fit/validation");
            }
            calibrated = true;
            calibrationStatus = "Fixed-speed trajectory loaded and validated";
        } catch (IOException | RuntimeException e) {
            calibrationStatus = "Math unavailable: " + e.getMessage();
        }
        status = calibrationStatus;
    }

    public boolean update(StationaryShotCue.Reading reading) {
        prediction = null;
        if (!calibrated) { status = calibrationStatus; return false; }
        if (!reading.allianceKnown || !Double.isFinite(reading.targetVelocity)
                || Math.abs(reading.targetVelocity-targetVelocity) > 1.0) {
            status = "Math needs the calibrated alliance and shooter setting";
            return false;
        }
        Opening first = null;
        int cell = -1;
        boolean allFit = true;
        for (StationaryShotCue.Target target : reading.targets) {
            if (target.id < (reading.redAlliance ? 30 : 38)
                    || target.id > (reading.redAlliance ? 37 : 45)) continue;
            Vec3 position = new Vec3(target.cameraX, target.cameraY, target.cameraZ);
            Rotation tagRotation = Rotation.yawPitchRoll(Math.toRadians(target.yaw),
                    Math.toRadians(target.pitch), Math.toRadians(target.roll));
            if (!position.finite() || !tagRotation.valid()) continue;
            int tagCell = (target.id-30)/4;
            int slot = (target.id-30)%4;
            double[] slotOffsets = {-6.5, -2.75, 2.75, 6.5};
            Vec3 offset = tagBottom.minus(new Vec3(slotOffsets[slot],0,0));
            Vec3 bottom = cameraOffset.plus(cameraRotation.apply(position.plus(tagRotation.apply(offset))));
            Vec3 right = cameraRotation.apply(tagRotation.apply(tagRight));
            Vec3 up = cameraRotation.apply(tagRotation.apply(tagUp));
            Opening opening = Opening.bioBuzz(bottom,right,up);
            // Only the upward, scorable cell; reject a visible downward cell or a bad frame.
            if (!opening.valid() || opening.outward.z <= 0.1
                    || Math.abs(bottom.z-StationaryShotCue.HIVE_OPENING_BOTTOM_INCHES) > 3.0
                    || Math.abs(bottom.plus(up.times(14)).z-StationaryShotCue.HIVE_OPENING_TOP_INCHES) > 3.0) continue;
            if (first != null && (tagCell != cell || bottom.minus(first.bottom).length() > 3.0
                    || right.dot(first.right) < Math.cos(Math.toRadians(6))
                    || up.dot(first.up) < Math.cos(Math.toRadians(6)))) {
                status = "Math: hive measurements disagree; wait for a clearer view";
                prediction = null;
                return false;
            }
            if (first == null) {
                first = opening;
                cell = tagCell;
            }
            Prediction candidate = predict(release,launch,opening,radius,margin,minDistance,maxDistance);
            allFit &= candidate.fits;
            if (prediction == null || !candidate.fits
                    || candidate.clearanceInches < prediction.clearanceInches) prediction = candidate;
        }
        if (prediction == null) {
            status = "Math needs a fresh, upward-cell 3D pose";
            return false;
        }
        status = prediction.reason;
        return allFit;
    }

    public void invalidate() {
        calibrated = false;
        prediction = null;
        data.setProperty("trajectory.validated", "false");
        data.setProperty("camera.verified", "false");
        data.setProperty("opening.verified", "false");
        calibrationStatus = "Math calibration cleared";
        try { store(); }
        catch (IOException | RuntimeException e) { calibrationStatus = "Math reset in memory; save failed: " + e.getMessage(); }
        status = calibrationStatus;
    }

    private void defaults() {
        data.clear();
        data.setProperty("version","1");
        data.setProperty("fieldGeometry",StationaryShotCue.FIELD_GEOMETRY);
        data.setProperty("camera.verified","false");
        data.setProperty("opening.verified","false");
        data.setProperty("trajectory.validated","false");
        // Seth's existing measured positions: robot +X front, +Y left, +Z up.
        setVector("camera.position",-5,0.5,13.5);
        setVector("muzzle.position",-6.5,0,5);
        for (String key : new String[]{"camera.yawDegrees","camera.pitchDegrees","camera.rollDegrees",
                "trajectory.horizontalInchesPerSecond","trajectory.verticalInchesPerSecond",
                "trajectory.minimumDistanceInches","trajectory.maximumDistanceInches",
                "trajectory.maximumFitErrorInches"}) data.setProperty(key,"NaN");
        // Raw Limelight tag frame -> lower-center of mouth. Calibrate/verify this mapping.
        setVector("opening.bottomAtClusterCenter",Double.NaN,Double.NaN,Double.NaN);
        setVector("opening.right",Double.NaN,Double.NaN,Double.NaN);
        setVector("opening.up",Double.NaN,Double.NaN,Double.NaN);
        data.setProperty("trajectory.targetTicksPerSecond","NaN");
        data.setProperty("trajectory.robotYawDegrees","180");
        data.setProperty("trajectory.ballRadiusInches","1.4");
        data.setProperty("trajectory.errorMarginInches","2");
        data.setProperty("validation.shots","0");
        data.setProperty("validation.scored","0");
    }

    private double number(String key) {
        double value = Double.parseDouble(data.getProperty(key));
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Missing " + key);
        return value;
    }
    private Vec3 vector(String prefix) {
        return new Vec3(number(prefix+".x"),number(prefix+".y"),number(prefix+".z"));
    }
    private void setVector(String prefix, double x, double y, double z) {
        data.setProperty(prefix+".x",Double.toString(x));
        data.setProperty(prefix+".y",Double.toString(y));
        data.setProperty(prefix+".z",Double.toString(z));
    }
    private void store() throws IOException {
        if (file == null) throw new IOException("Model storage not initialized");
        File parent = file.getAbsoluteFile().getParentFile();
        if (!parent.isDirectory() && !parent.mkdirs()) throw new IOException("Cannot create model directory");
        try (FileOutputStream output = new FileOutputStream(file)) {
            data.store(output,"Stationary fixed-speed shot math. Unverified fields must be measured before activation.");
            output.getFD().sync();
        }
    }

    public boolean isCalibrated() { return calibrated; }
    public String getStatus() { return status; }
    public Prediction getPrediction() { return prediction; }
}
