package org.firstinspires.ftc.teamcode.Robot1.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot1.Init.TEST_CuttleInitOpMode;

import java.util.List;

public class Limelight extends TEST_CuttleInitOpMode {
    Limelight3A limelight;
    public Limelight(Limelight3A limelight){
        this.limelight = limelight;
    }

    public void initCam(int pipeline) {
        telemetry.setMsTransmissionInterval(11);

        pipeline(pipeline);

        //Starts polling for data
        limelight.start();
    }

    public void pipeline(CuttleIntake.Color color){
        if(color == CuttleIntake.Color.YELLOW){limelight.pipelineSwitch(0);}
        else if(color == CuttleIntake.Color.BLUE){limelight.pipelineSwitch(1);}
        else if(color == CuttleIntake.Color.RED){limelight.pipelineSwitch(2);}
        else{limelight.pipelineSwitch(3);}
    }

    public void pipeline(int pipeline){limelight.pipelineSwitch(pipeline);}

    public void off(){limelight.stop();}

    public void detect() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());

            }
        }
    }

    public void advancedDetection() {

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // print some data for each detected target
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
        }
    }

}
