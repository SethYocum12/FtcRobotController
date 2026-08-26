package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LIMELIGHT_setup {
    private Limelight3A limelight;

    double distance;
    LLResult llResult;

    boolean distanceTele;
    boolean tagIdTele;
    boolean txTele;
    boolean tyTele;
    boolean taTele;
    boolean noLimeTele;


    public void init(HardwareMap hwmap){
        try {
            limelight = hwmap.get(Limelight3A.class, "limelight");// finding april tag hw map
            limelight.pipelineSwitch(8);// finding pipeline
        } catch (Exception e) {
            limelight = null;
        }
    }

    public double getDistanceFromTag(double ta) {
        // Area decreases with the square of distance, so we use sqrt(ta) to linearize it.
        // Final calibration: (49.5 actual / 50.0 reported) * 180.4 = 178.6
        double scale = 178.6;
        return (scale / Math.sqrt(ta));
    }

    public void main(){
        if (limelight == null) {
            noLimeTele = true;
            return;
        }
        llResult = limelight.getLatestResult();// pulls data from limelight
        if (llResult != null && llResult.isValid()) { // if the result is a thing and is valid do:
            distance = getDistanceFromTag((llResult.getTa())); //gets the distances from the tag
            distanceTele = true;
            // telemetry.addData("distance", distance);
            // Check for the specific AprilTag ID. fidual is fancy word for april tag
            if (!llResult.getFiducialResults().isEmpty()) { //if the tag is not empty, return tag ID
                tagIdTele = true;
                // telemetry.addData("Tag ID", llResult.getFiducialResults().get(0).getFiducialId());
            } else {
                tagIdTele = false;
            }

            tyTele = true;
            txTele = true;
            taTele = true;
            noLimeTele = false;
            //telemetry.addData("Target x", llResult.getTx());
            //telemetry.addData("Target y", llResult.getTy());
            //telemetry.addData("Target Area", llResult.getTa());
        } else {
            distanceTele = false;
            tyTele = false;
            txTele = false;
            taTele = false;
            noLimeTele = true;
        }
    }


    public double returnDistance(){
        return distance;
    }
    public double returnLLResultTx(){
        if (llResult == null) return 0.0;
        return llResult.getTx();
    }

    public double returnLLResultTy(){
        if (llResult == null) return 0.0;
        return llResult.getTy();
    }

    public double returnLLResultTa(){
        if (llResult == null) return 0.0;
        return llResult.getTa();
    }

    public double returnTagId(){
        if (llResult == null || llResult.getFiducialResults().isEmpty()) return 0.0;
        return llResult.getFiducialResults().get(0).getFiducialId();
    }

    public boolean distanceTele(){
        return distanceTele;
    }

    public boolean tagIdTele(){
        return tagIdTele;
    }

    public boolean txTele(){
        return txTele;
    }

    public boolean tyTele(){
        return tyTele;
    }

    public boolean taTele(){
        return taTele;
    }

    public boolean noLimeTele(){
        return noLimeTele;
    }



    public void limelight_start(){
        limelight.start();
    }

}


