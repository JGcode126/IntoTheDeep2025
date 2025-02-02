package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboctopi.cuttlefish.queue.TaskList;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous
@Config
public class _5Spec extends CuttleInitOpModeRobot2 {
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

        auto.startSpecimen(80, -860, 2.7, 0.7, 0.6, -650, -800, 80,1.0);

        specimen.ttSampleOther(1.0,4,-705,-860 ,90,-760,-690 ,60, -0.3);//first sample
        //x1: -755, y1 used to be -700
        specimen.ttSampleOther(5,5,-760,-690,90,-780,-700,50, -0.3);//second sample
        specimen.ttSample(3,6, -1100,-780,135,-1000,-600,50,2.0);//third sample

        //-450+offsetx, -500+offsety, Math.toRadians(60+offsetr))
        specimen.scoringSpecimenFancy(0, 10, 100,0,100, -850);//1st specimen
        specimen.scoringSpecimenFancy(0.25, -6,100,-20,120,-850);//2nd specimen
        specimen.scoringSpecimenFancy(0.75,-7,100, -20,150,-850);//3rd specimen
        specimen.scoringSpecimenFancy(1.0,-10,100, -20,180,-850);//3rd specimen

        specimen.specimenPark(1);
    }

    public void mainLoop() {
        super.mainLoop();
        loopCounter = specimen.yellowPark(loopCounter);
        setup.telemetryData();
    }
}
