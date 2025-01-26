package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous
@Config
public class SpecimenAuto extends CuttleInitOpModeRobot2 {
    public int loopCounter = 0;
    public static int distance = -850;

    public static int x = 0;
    public static int y = -360;
    public static int r = 0;

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

        auto.startSpecimen(distance, 2, 0.6, 0.7);
        //auto.allSamples();

        //specimen.scoringSpecimenFancy(0, -4, 0,0,0);//1st specimen
        //specimen.scoringSpecimenFancy(0.25, -6,0,0,20);//2nd specimen
        //specimen.scoringSpecimenFancy(0.75,-7,0, 0,50);//3rd specimen
        //specimen.scoringSpecimenFancy(1.5,-9,0, 0,70);//4th specimen

        //specimen.specimenPark(1);
    }

    public void mainLoop() {
        super.mainLoop();
        loopCounter = specimen.yellowPark(loopCounter);
        setup.telemetryData();
    }
}
