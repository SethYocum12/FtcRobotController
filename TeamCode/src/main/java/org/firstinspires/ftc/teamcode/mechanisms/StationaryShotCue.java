package org.firstinspires.ftc.teamcode.mechanisms;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Properties;

/** Observes manual shots. This class has no access to motors, servos, or gamepads. */
public final class StationaryShotCue {
    // FIRST BIOBUZZ Arena TU06, Figures 9-10, 9-11 and 9-15 (pages 71, 72, 75).
    // Geometry identifies the field reference set; it does not determine ball trajectory.
    public static final String FIELD_GEOMETRY = "BIOBUZZ-ARENA-TU06";
    public static final double TAG_SIZE_INCHES = 3.25;
    public static final double HIVE_OPENING_WIDTH_INCHES = 20.0;
    public static final double HIVE_OPENING_FACE_HEIGHT_INCHES = 14.0;
    public static final double HIVE_OPENING_BOTTOM_INCHES = 53.5;
    public static final double HIVE_OPENING_TOP_INCHES = 65.6;
    public static final double MAX_SPEED_INCHES_PER_SECOND = 1.0;
    public static final double MAX_TURN_RADIANS_PER_SECOND = Math.toRadians(3.0);
    public static final double STILL_SECONDS = 0.4;
    public static final double ANGLE_WINDOW_DEGREES = 1.5;
    public static final double RANGE_WINDOW_INCHES = 2.0;
    public static final double APPROACH_WINDOW_DEGREES = 6.0;
    private static final double MAX_LOOP_GAP_SECONDS = 0.25;
    private static final double TARGET_WINDOW_TICKS_PER_SECOND = 1.0;
    private static final int MAX_REFERENCES = 256;

    /** One fresh tag: angles and 3D distance must all come from this same tag. */
    public static final class Target {
        public final int id;
        public final double tx, ty, rangeInches;
        public final double yaw, pitch, roll;
        public final double cameraX, cameraY, cameraZ;

        public Target(int id, double tx, double ty, double rangeInches,
                      double yaw, double pitch, double roll) {
            this(id,tx,ty,rangeInches,yaw,pitch,roll,Double.NaN,Double.NaN,Double.NaN);
        }

        public Target(int id, double tx, double ty, double rangeInches,
                      double yaw, double pitch, double roll, double cameraX, double cameraY, double cameraZ) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.rangeInches = rangeInches;
            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;
            this.cameraX = cameraX;
            this.cameraY = cameraY;
            this.cameraZ = cameraZ;
        }

        private boolean validFor(boolean redAlliance) {
            return id >= (redAlliance ? 30 : 38) && id <= (redAlliance ? 37 : 45)
                    && Double.isFinite(tx) && Double.isFinite(ty)
                    && Double.isFinite(yaw) && Double.isFinite(pitch) && Double.isFinite(roll)
                    && Double.isFinite(rangeInches) && rangeInches > 0;
        }

        public boolean hasCameraPose() {
            return Double.isFinite(cameraX) && Double.isFinite(cameraY) && Double.isFinite(cameraZ)
                    && Double.isFinite(yaw) && Double.isFinite(pitch) && Double.isFinite(roll)
                    && Double.isFinite(rangeInches) && rangeInches > 0;
        }
    }

    public static final class Reading {
        public double seconds = Double.NaN;
        public boolean allianceKnown, redAlliance, driveRequested;
        public boolean shooterOn, flywheelReady, feeding;
        public boolean reversing;
        public double targetVelocity = Double.NaN;
        public double actualVelocity = Double.NaN, batteryVoltage = Double.NaN, visionAgeMs = Double.NaN;
        public double robotX = Double.NaN, robotY = Double.NaN, robotHeading = Double.NaN;
        public double vx = Double.NaN, vy = Double.NaN, omega = Double.NaN;
        public List<Target> targets = Collections.emptyList();
    }

    private static final class Reference {
        final boolean redAlliance;
        final double velocity;
        final Target target;

        Reference(boolean redAlliance, double velocity, Target target) {
            this.redAlliance = redAlliance;
            this.velocity = velocity;
            this.target = target;
        }
    }

    private final List<Reference> references = new ArrayList<>();
    private File file;
    private double previousTime = Double.NaN;
    private double stillSince = Double.NaN;
    private boolean settled;
    private boolean ready;
    private boolean operatingReady;
    private String operatingStatus = "Waiting for stationary shot checks";
    private String status = "Waiting for stationary shot checks";
    private String saveStatus = "No confirmed shot saved";

    /** References live in the Control Hub's FIRST/settings directory, across OpModes. */
    public void load(File file) {
        this.file = file;
        references.clear();
        reset();
        if (!file.isFile()) {
            saveStatus = "No confirmed shot saved";
            return;
        }
        try (FileInputStream input = new FileInputStream(file)) {
            Properties data = new Properties();
            data.load(input);
            if (!"1".equals(data.getProperty("version"))) throw new IOException("Unknown format");
            if (!FIELD_GEOMETRY.equals(data.getProperty("fieldGeometry"))) {
                throw new IOException("Confirm references for the real BIOBUZZ hive");
            }
            int count = Integer.parseInt(data.getProperty("count"));
            if (count < 0 || count > MAX_REFERENCES) throw new IOException("Invalid count");
            List<Reference> loaded = new ArrayList<>();
            for (int i = 0; i < count; i++) {
                String prefix = "shot." + i + ".";
                String alliance = data.getProperty(prefix + "alliance");
                if (!"RED".equals(alliance) && !"BLUE".equals(alliance)) {
                    throw new IOException("Invalid alliance");
                }
                boolean red = "RED".equals(alliance);
                double velocity = Double.parseDouble(data.getProperty(prefix + "velocity"));
                Target target = new Target(Integer.parseInt(data.getProperty(prefix + "tag")),
                        Double.parseDouble(data.getProperty(prefix + "tx")),
                        Double.parseDouble(data.getProperty(prefix + "ty")),
                        Double.parseDouble(data.getProperty(prefix + "rangeInches")),
                        Double.parseDouble(data.getProperty(prefix + "yaw")),
                        Double.parseDouble(data.getProperty(prefix + "pitch")),
                        Double.parseDouble(data.getProperty(prefix + "roll")));
                if (!target.validFor(red) || !Double.isFinite(velocity) || velocity <= 0) {
                    throw new IOException("Invalid saved reading");
                }
                loaded.add(new Reference(red, velocity, target));
            }
            references.addAll(loaded);
            saveStatus = count == 0 ? "No confirmed shot saved" : "Loaded " + count + " tag references";
        } catch (IOException | RuntimeException e) {
            saveStatus = "Saved shots unreadable; confirm a new shot";
        }
    }

    public void reset() {
        previousTime = Double.NaN;
        stillSince = Double.NaN;
        settled = false;
        ready = false;
        operatingReady = false;
        operatingStatus = "Waiting for stationary shot checks";
        status = "Waiting for stationary shot checks";
    }

    /** Always evaluates the shot; its output controls rumble only. */
    public boolean update(Reading reading) {
        ready = false;
        operatingReady = false;
        boolean timingOK = Double.isFinite(reading.seconds)
                && Double.isFinite(previousTime) && reading.seconds >= previousTime
                && reading.seconds - previousTime <= MAX_LOOP_GAP_SECONDS;
        if (!timingOK) stillSince = Double.NaN;
        previousTime = reading.seconds;
        boolean still = !reading.driveRequested && Double.isFinite(reading.seconds)
                && Double.isFinite(reading.vx) && Double.isFinite(reading.vy)
                && Double.isFinite(reading.omega)
                && Math.hypot(reading.vx, reading.vy) <= MAX_SPEED_INCHES_PER_SECOND
                && Math.abs(reading.omega) <= MAX_TURN_RADIANS_PER_SECOND;
        if (!still) stillSince = Double.NaN;
        else if (!Double.isFinite(stillSince)) stillSince = reading.seconds;
        settled = still && reading.seconds - stillSince >= STILL_SECONDS;

        String reason = missingRequirement(reading);
        operatingStatus = reason;
        if (reason != null) {
            status = reason;
            return false;
        }
        operatingReady = true;
        if (references.isEmpty()) {
            status = "No confirmed shot saved";
            return false;
        }
        for (Reference reference : references) {
            if (reference.redAlliance != reading.redAlliance
                    || Math.abs(reference.velocity - reading.targetVelocity)
                    > TARGET_WINDOW_TICKS_PER_SECOND) continue;
            for (Target target : reading.targets) {
                if (target.validFor(reading.redAlliance) && target.id == reference.target.id
                        && Math.abs(target.tx - reference.target.tx) <= ANGLE_WINDOW_DEGREES
                        && Math.abs(target.ty - reference.target.ty) <= ANGLE_WINDOW_DEGREES
                        && Math.abs(target.rangeInches - reference.target.rangeInches)
                        <= RANGE_WINDOW_INCHES
                        && angleDistance(target.yaw, reference.target.yaw) <= APPROACH_WINDOW_DEGREES
                        && angleDistance(target.pitch, reference.target.pitch) <= APPROACH_WINDOW_DEGREES
                        && angleDistance(target.roll, reference.target.roll) <= APPROACH_WINDOW_DEGREES) {
                    ready = true;
                    status = "Matches confirmed shot: tag " + target.id;
                    return true;
                }
            }
        }
        status = "Match a saved angle, distance, approach and shooter setting";
        return false;
    }

    private String missingRequirement(Reading reading) {
        if (!reading.allianceKnown) return "Select alliance in INIT";
        if (!settled) return "Stop driving/turning; settle for 0.4 s";
        if (!reading.shooterOn) return "Turn shooter on with Triangle / Y";
        if (!Double.isFinite(reading.targetVelocity) || reading.targetVelocity <= 0) {
            return "Invalid shooter setting";
        }
        if (!reading.flywheelReady) return "Wait for flywheel speed/recovery";
        for (Target target : reading.targets) {
            if (target.validFor(reading.redAlliance)) return null;
        }
        return "Need fresh alliance tag with full 3D pose";
    }

    /** Driver confirms scoring manually; neither a tag nor a speed drop confirms a basket. */
    public boolean confirmScoringPosition(Reading reading) {
        String reason = missingRequirement(reading);
        if (Double.compare(reading.seconds, previousTime) != 0) reason = "Wait for a fresh reading";
        if (reading.feeding) reason = "Stop feeding with Circle / B before saving";
        if (reason != null) {
            saveStatus = "Not saved: " + reason;
            return false;
        }
        List<Reference> updated = new ArrayList<>(references);
        int count = 0;
        for (Target target : reading.targets) {
            if (!target.validFor(reading.redAlliance)) continue;
            // Replace repeat captures of the same spot; retain other tested spots.
            updated.removeIf(old -> old.redAlliance == reading.redAlliance
                    && old.target.id == target.id
                    && Math.abs(old.velocity - reading.targetVelocity) <= TARGET_WINDOW_TICKS_PER_SECOND
                    && Math.abs(old.target.tx - target.tx) < 0.25
                    && Math.abs(old.target.ty - target.ty) < 0.25
                    && Math.abs(old.target.rangeInches - target.rangeInches) < 0.5
                    && angleDistance(old.target.yaw, target.yaw) < 1
                    && angleDistance(old.target.pitch, target.pitch) < 1
                    && angleDistance(old.target.roll, target.roll) < 1);
            updated.add(new Reference(reading.redAlliance, reading.targetVelocity, target));
            count++;
        }
        if (updated.size() > MAX_REFERENCES) {
            saveStatus = "Not saved: reference limit reached";
            return false;
        }
        if (!write(updated)) return false;
        references.clear();
        references.addAll(updated);
        saveStatus = "Saved confirmed position: " + count + " tag references";
        return true;
    }

    public boolean clear() {
        if (!write(Collections.emptyList())) return false;
        references.clear();
        reset();
        saveStatus = "Cleared confirmed shot references";
        return true;
    }

    private boolean write(List<Reference> updated) {
        File temporary = null;
        try {
            if (file == null) throw new IOException("Storage not initialized");
            File parent = file.getAbsoluteFile().getParentFile();
            if (!parent.isDirectory() && !parent.mkdirs()) throw new IOException("Cannot create directory");
            Properties data = new Properties();
            data.setProperty("version", "1");
            data.setProperty("fieldGeometry", FIELD_GEOMETRY);
            data.setProperty("geometry.tagSizeInches", Double.toString(TAG_SIZE_INCHES));
            data.setProperty("geometry.openingWidthInches", Double.toString(HIVE_OPENING_WIDTH_INCHES));
            data.setProperty("geometry.openingFaceHeightInches", Double.toString(HIVE_OPENING_FACE_HEIGHT_INCHES));
            data.setProperty("geometry.openingBottomInches", Double.toString(HIVE_OPENING_BOTTOM_INCHES));
            data.setProperty("geometry.openingTopInches", Double.toString(HIVE_OPENING_TOP_INCHES));
            data.setProperty("count", Integer.toString(updated.size()));
            for (int i = 0; i < updated.size(); i++) {
                Reference reference = updated.get(i);
                String prefix = "shot." + i + ".";
                data.setProperty(prefix + "alliance", reference.redAlliance ? "RED" : "BLUE");
                data.setProperty(prefix + "velocity", Double.toString(reference.velocity));
                data.setProperty(prefix + "tag", Integer.toString(reference.target.id));
                data.setProperty(prefix + "tx", Double.toString(reference.target.tx));
                data.setProperty(prefix + "ty", Double.toString(reference.target.ty));
                data.setProperty(prefix + "rangeInches", Double.toString(reference.target.rangeInches));
                data.setProperty(prefix + "yaw", Double.toString(reference.target.yaw));
                data.setProperty(prefix + "pitch", Double.toString(reference.target.pitch));
                data.setProperty(prefix + "roll", Double.toString(reference.target.roll));
            }
            temporary = File.createTempFile("shot-cue-", ".tmp", parent);
            try (FileOutputStream output = new FileOutputStream(temporary)) {
                data.store(output, "Manually confirmed stationary shots; per-tag camera readings");
                output.getFD().sync();
            }
            if (!temporary.renameTo(file.getAbsoluteFile())) throw new IOException("Cannot replace saved file");
            return true;
        } catch (IOException | RuntimeException e) {
            saveStatus = "Save failed: " + e.getMessage();
            return false;
        } finally {
            if (temporary != null && temporary.exists()) temporary.delete();
        }
    }

    private static double angleDistance(double first, double second) {
        return Math.abs(Math.IEEEremainder(first - second, 360.0));
    }

    public boolean isReady() { return ready; }
    public boolean isOperatingReady() { return operatingReady; }
    public String getOperatingStatus() { return operatingStatus; }
    public String getStatus() { return status; }
    public String getSaveStatus() { return saveStatus; }
    public int getReferenceCount() { return references.size(); }
}
