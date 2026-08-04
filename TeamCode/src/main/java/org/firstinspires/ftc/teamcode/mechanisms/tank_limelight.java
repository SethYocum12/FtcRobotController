package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class tank_limelight {

    private Limelight3A limelight;

    public void init(HardwareMap hwmap){
        limelight = hwmap.get(Limelight3A.class,"limelight");// finding april tag hw map
        limelight.pipelineSwitch(8);// finding pipeline
    }
}


