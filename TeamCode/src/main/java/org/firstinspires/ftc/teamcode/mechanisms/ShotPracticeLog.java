package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/** One durable CSV row per deliberate button press, not a loop-rate file writer. */
public class ShotPracticeLog {
    private File file;
    private int samples;
    private String status = "No samples yet";

    public void record(boolean allianceKnown, boolean red, AimAssist aim,
                       double target, double actual, double batteryVolts,
                       Pose pose, Velocity velocity, boolean aiming, boolean feeding) {
        if (!allianceKnown || !aim.hasAimTarget(red)
                || !Double.isFinite(aim.getTagRangeInches(red))) {
            status = "Not saved: need alliance and fresh tag with range";
            return;
        }
        try {
            if (file == null) {
                File directory = new File(AppUtil.FIRST_FOLDER, "shot-practice");
                if (!directory.isDirectory() && !directory.mkdirs())
                    throw new IOException("Cannot create log directory");
                String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss-SSS", Locale.US)
                        .format(new Date());
                file = new File(directory, "shots-" + stamp + ".csv");
            }
            boolean header = !file.exists() || file.length() == 0;
            try (FileWriter writer = new FileWriter(file, true)) {
                if (header) writer.write("sample,time_ms,alliance,tag_id,camera_tag_range_in,"
                        + "raw_ty_deg,aim_error_deg,target_ticks_s,actual_ticks_s,battery_v,"
                        + "pose_x_in,pose_y_in,heading_rad,vx_in_s,vy_in_s,omega_rad_s,aiming,feeding\n");
                writer.write(String.format(Locale.US,
                        "%d,%d,%s,%d,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%b,%b%n",
                        samples + 1, System.currentTimeMillis(), red ? "RED" : "BLUE",
                        aim.getAimTagId(red), aim.getTagRangeInches(red), aim.getRawAimError(red),
                        aim.getAimError(red), target, actual, batteryVolts,
                        pose.x(), pose.y(), pose.heading(), velocity.vx, velocity.vy, velocity.omega,
                        aiming, feeding));
            }
            samples++;
            status = "Saved sample " + samples;
        } catch (IOException | SecurityException error) {
            status = "Save failed: " + error.getMessage();
        }
    }

    public int getSamples() { return samples; }
    public String getStatus() { return status; }
    public String getPath() { return file == null ? "Created on first valid sample" : file.getAbsolutePath(); }
}
