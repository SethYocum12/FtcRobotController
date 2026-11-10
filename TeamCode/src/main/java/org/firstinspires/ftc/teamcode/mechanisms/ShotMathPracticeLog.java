package org.firstinspires.ftc.teamcode.mechanisms;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/** Practice-only CSV. Frames and feed commands are observations, not counted shots. */
public final class ShotMathPracticeLog {
    private FileOutputStream output;
    private BufferedWriter writer;
    private File file;
    private int batch, frame;
    private double lastSample = Double.NEGATIVE_INFINITY;
    private double lastFlush = Double.NEGATIVE_INFINITY;
    private String status = "Practice recording off";

    public void start(File directory) {
        finish();
        batch = frame = 0;
        lastSample = lastFlush = Double.NEGATIVE_INFINITY;
        try {
            if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create log directory");
            String date = new SimpleDateFormat("yyyyMMdd-HHmmss-SSS", Locale.US).format(new Date());
            file = File.createTempFile("shot-math-" + date + "-", ".csv", directory);
            output = new FileOutputStream(file);
            writer = new BufferedWriter(new OutputStreamWriter(output, StandardCharsets.UTF_8));
            writer.write("event,batch,frame,utc_ms,seconds,alliance,tag,tx_deg,ty_deg,range_in,"
                    + "tag_camera_x_in,tag_camera_y_in,tag_camera_z_in,tag_yaw_deg,tag_pitch_deg,tag_roll_deg,"
                    + "robot_x_in,robot_y_in,robot_heading_rad,vx_in_s,vy_in_s,omega_rad_s,"
                    + "target_ticks_s,actual_ticks_s,flywheel_ready,feed_requested,reverse_requested,"
                    + "vision_age_ms,drive_requested,battery_v\n");
            flush();
            status = "Recording; Down labels position 1";
        } catch (IOException | RuntimeException e) { fail(e); }
    }

    public void markPosition(StationaryShotCue.Reading reading) {
        if (writer == null) return;
        batch++;
        append("position",reading);
        if (writer != null) status = "Recording position " + batch;
    }

    public void markConfirmed(StationaryShotCue.Reading reading) {
        if (writer != null) append("confirmed_position",reading);
    }

    public void sample(StationaryShotCue.Reading reading) {
        if (writer == null || reading.seconds-lastSample < 0.1) return;
        append("frame",reading);
    }

    private void append(String event, StationaryShotCue.Reading r) {
        frame++;
        lastSample = r.seconds;
        try {
            if (r.targets.isEmpty()) row(event,r,null);
            else for (StationaryShotCue.Target target : r.targets) row(event,r,target);
            if (r.seconds-lastFlush >= 1.0 || !"frame".equals(event)) {
                flush();
                lastFlush = r.seconds;
            }
        } catch (IOException | RuntimeException e) { fail(e); }
    }

    private void row(String event, StationaryShotCue.Reading r, StationaryShotCue.Target tag) throws IOException {
        String alliance = r.allianceKnown ? (r.redAlliance ? "RED" : "BLUE") : "UNKNOWN";
        StringBuilder line = new StringBuilder(event).append(',').append(batch).append(',').append(frame)
                .append(',').append(System.currentTimeMillis()).append(',').append(r.seconds).append(',').append(alliance);
        line.append(',').append(tag == null ? -1 : tag.id);
        double[] values = tag == null
                ? new double[]{Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN}
                : new double[]{tag.tx,tag.ty,tag.rangeInches,tag.cameraX,tag.cameraY,tag.cameraZ,tag.yaw,tag.pitch,tag.roll};
        for (double value : values) line.append(',').append(value);
        for (double value : new double[]{r.robotX,r.robotY,r.robotHeading,r.vx,r.vy,r.omega,
                r.targetVelocity,r.actualVelocity}) line.append(',').append(value);
        line.append(',').append(r.flywheelReady).append(',').append(r.feeding).append(',').append(r.reversing)
                .append(',').append(r.visionAgeMs).append(',').append(r.driveRequested)
                .append(',').append(r.batteryVoltage).append('\n');
        writer.write(line.toString());
    }

    private void flush() throws IOException {
        writer.flush();
        output.getFD().sync();
    }

    private void fail(Exception error) {
        String message = "Recording failed: " + error.getMessage();
        finish();
        status = message;
    }

    public void finish() {
        if (writer == null) return;
        try {
            flush();
            writer.close();
            status = "Saved " + frame + " frames; " + batch + " labeled positions";
        } catch (IOException e) {
            status = "Log close failed: " + e.getMessage();
        } finally {
            writer = null;
            if (output != null) try { output.close(); } catch (IOException ignored) { }
            output = null;
        }
    }

    public int getBatch() { return batch; }
    public String getStatus() { return status; }
    public String getPath() { return file == null ? "Not recording" : file.getAbsolutePath(); }
}
