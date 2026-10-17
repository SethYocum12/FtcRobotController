package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Shared Limelight access for teleop and autonomous. Call main() each loop. */
public class LIMELIGHT_setup {
    public static final int APRILTAG_PIPELINE = 8;
    private static final long MAX_RESULT_AGE_MS = 250;
    private Limelight3A limelight;
    private LLResult llResult;
    private String status = "Not initialized";
    private double distance;
    private boolean distanceTele;
    private boolean tagIdTele;
    private boolean txTele;
    private boolean tyTele;
    private boolean taTele;
    private boolean noLimeTele = true;

    public void init(HardwareMap hwmap) {
        limelight_stop();
        try {
            limelight = hwmap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            status = limelight.pipelineSwitch(APRILTAG_PIPELINE)
                    ? "Ready to start" : "Pipeline switch failed";
        } catch (RuntimeException e) {
            if (limelight != null) limelight.stop();
            limelight = null;
            status = "Unavailable: " + e.getMessage();
        }
    }

    public void limelight_start() {
        if (limelight != null) {
            limelight.start();
        }
    }

    public void limelight_stop() {
        if (limelight != null) limelight.stop();
        llResult = null;
        clearDetection();
        status = "Stopped";
    }

    private void clearDetection() {
        distance = 0;
        distanceTele = false;
        tagIdTele = false;
        txTele = false;
        tyTele = false;
        taTele = false;
        noLimeTele = true;
    }

    public void main() {
        clearDetection();
        if (limelight == null) {
            llResult = null;
            return;
        }
        llResult = limelight.getLatestResult();
        if (!limelight.isRunning()) {
            status = "Polling stopped";
        } else if (!limelight.isConnected()) {
            status = "Disconnected or waiting for frames";
        } else if (llResult == null) {
            status = "Waiting for frames";
        } else if (llResult.getStaleness() > MAX_RESULT_AGE_MS) {
            status = "Stale data";
        } else if (llResult.getPipelineIndex() != APRILTAG_PIPELINE) {
            status = "Wrong pipeline";
        } else if (!llResult.isValid() || llResult.getFiducialResults().isEmpty()) {
            status = "No AprilTag detected";
        } else {
            status = "AprilTag detected";
            tagIdTele = true;
            txTele = true;
            tyTele = true;
            taTele = true;
            noLimeTele = false;
            double estimatedDistance = getDistanceFromTag(llResult.getTa());
            if (Double.isFinite(estimatedDistance) && estimatedDistance > 0) {
                distance = estimatedDistance;
                distanceTele = true;
            }
        }
    }

    private boolean hasFreshTag() {
        return limelight != null && limelight.isRunning() && limelight.isConnected()
                && llResult != null && llResult.getStaleness() <= MAX_RESULT_AGE_MS
                && llResult.getPipelineIndex() == APRILTAG_PIPELINE
                && llResult.isValid() && !llResult.getFiducialResults().isEmpty();
    }

    /** Checks every detected ID. A missing tag alone does not establish a hive tip. */
    public boolean hasTag(int expectedId) {
        if (!hasFreshTag()) return false;
        for (LLResultTypes.FiducialResult tag : llResult.getFiducialResults()) {
            if (tag.getFiducialId() == expectedId) return true;
        }
        return false;
    }

    public String getTagIds() {
        if (!hasFreshTag()) return "None";
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : llResult.getFiducialResults()) {
            if (ids.length() > 0) ids.append(", ");
            ids.append(tag.getFiducialId());
        }
        return ids.toString();
    }

    /** Empirical estimate for the original calibration target, not general 3D range. */
    public double getDistanceFromTag(double ta) {
        if (!Double.isFinite(ta) || ta <= 0) return 0;
        return 178.6 / Math.sqrt(ta);
    }

    public double returnDistance() { return distanceTele() ? distance : 0; }
    // TX/TY refer to the pipeline's primary target, not a tag selected by hasTag().
    public double returnLLResultTx() { return hasFreshTag() ? llResult.getTx() : 0; }
    public double returnLLResultTy() { return hasFreshTag() ? llResult.getTy() : 0; }
    public double returnLLResultTa() { return hasFreshTag() ? llResult.getTa() : 0; }

    /** Returns the first detected ID for compatibility with existing teleop telemetry. */
    public double returnTagId() {
        return hasFreshTag() ? llResult.getFiducialResults().get(0).getFiducialId() : 0;
    }

    public boolean distanceTele() { return distanceTele && hasFreshTag(); }
    public boolean tagIdTele() { return tagIdTele && hasFreshTag(); }
    public boolean txTele() { return txTele && hasFreshTag(); }
    public boolean tyTele() { return tyTele && hasFreshTag(); }
    public boolean taTele() { return taTele && hasFreshTag(); }
    public boolean noLimeTele() { return noLimeTele || !hasFreshTag(); }
    public String getStatus() { return status; }
    public boolean isConnected() { return limelight != null && limelight.isConnected(); }
    public long getResultAgeMs() { return llResult == null ? -1 : llResult.getStaleness(); }
    public int getPipelineIndex() { return llResult == null ? -1 : llResult.getPipelineIndex(); }
    public String getPipelineType() { return llResult == null ? "Unknown" : llResult.getPipelineType(); }
}
