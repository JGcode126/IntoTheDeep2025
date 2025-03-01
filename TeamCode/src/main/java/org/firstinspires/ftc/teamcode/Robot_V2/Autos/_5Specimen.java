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
        specimen.sweepSetup(510,600,-30);
        specimen.sampleSweep(4.9, 500,600,-30,770,0.35,0.3);

        specimen.sweepSetup(770,670,-40);
        specimen.sampleSweep(5.2, 770,670,-40,730,0.35,0.3);

        specimen.sweepSetup(975,870,-60);
        specimen.sampleSweep(4.8, 970,870,-60,750,0.35,0.3);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 1000,50,0,0.4);
        specimen.score(-50,600,0,-50,950,0,0.8,0.5);//-30

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 650,70,0,0.4);
        specimen.score(-120,600,0,-100,950,0,0.8,0.5);//-90

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 650,70,0,0.4);
        specimen.score(-170,600,0,-150,950,0,0.8,0.5);//-140

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 650,70,0,0.4);
        specimen.score(-230,600,0,-200,950,0,0.8,0.5);//-230

        specimen.specimenPark(1);
        //specimen.extendoPark();
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
