package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous
@Config
public class NEW_SPEC extends CuttleInitOpModeRobot2 {
    public int loopCounter = 0;

    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();
        setup.test = false;
        setup.side = "right";
        setup.color = "blue";
    }

    public void main(){
        super.main();
        loopCounter = 0;

        auto.startSpecimen(80, -860, 2, 0.7, 0.6, -650, -800, 80,1.0);

        /*specimen.ttSampleOther(1.0,4,-705,-860 ,90,-760,-690 ,60, -0.3);//first sample
        specimen.ttSampleOther(5,5,-760,-690,90,-780,-700,50, -0.3);//second sample*/
        //specimen.ttSample(3,6, -1100,-780,135,-1000,-600,50,2.0);//third sample

        specimen.scoreSetup();
        specimen.scoring(-900,-400,200, -900,-100,200);
        specimen.score(0,-800,200);

    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
