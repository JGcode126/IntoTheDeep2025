package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous(name = "specimen_5", group = "Example")
@Config
public class _5Specimen extends CuttleInitOpModeRobot2 {
    public int loopCounter = 0;

    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();
        setup.test = false;
        setup.side = "right";
        setup.color = "blue";

        bucketAuto = false;
    }

    public void main(){
        super.main();
        loopCounter = 0;

        specimen.scoreOther(-130,1000,0, 0.6,0.4);
        /*specimen.sweepSetup(500,600,-15,0.8);
        specimen.sampleSweep(2.6, 500,600,-15,1050,0.35,0.25);

        specimen.sweepSetup(770,670,-30,0.8);
        specimen.sampleSweep(4.8, 770,670,-30,920,0.35,0.3);

        specimen.sweepSetup(970,870,-60,0.8);
        specimen.sampleSweep(4.5, 970,870,-60,920,0.35,0.3);*/

        //900,900,950
        specimen.sweepSetup(520,610,-30);
        specimen.sampleSweep(4.5, 510,610,-30,850,0.35,0.3);

        specimen.sweepSetup(785,695,-40);
        specimen.sampleSweep(4.5, 785,695,-40,850,0.35,0.3);

        specimen.sweepSetup(985,880,-60);
        specimen.sampleSweep(4.5, 985,880,-60,900,0.35,0.3);

        specimen.scoreSetup();

        specimen.intakeOffWall(900,400,0, 1000,50,0,0.4);
        specimen.score(0,600,0,0.8);//-30

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 650,70,0,0.4);
        specimen.score(-60,600,0,0.8);//-90

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 650,70,0,0.4);
        specimen.score(-120,600,0,0.8);//-140

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 650,70,0,0.4);
        specimen.score(-180,600,0,0.8);//-230

        specimen.specimenPark(1);
        //specimen.extendoPark();
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
