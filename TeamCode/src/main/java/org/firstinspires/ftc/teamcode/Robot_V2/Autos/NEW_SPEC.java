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

        specimen.scoreOther(-150,950,0, 0.6);

        specimen.sweepSetup(500,600,-30);
        specimen.sampleSweep(4.9, 500,600,-30,500,600,-110, 600);

        specimen.sweepSetup(770,670,-40);
        specimen.sampleSweep(5.2, 770,670,-40,770,670,-110, 600);

        specimen.sweepSetup(970,870,-60);
        specimen.sampleSweep(4.8, 970,870,-60,970,870,-110, 700);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,300,0, 920,50,0);
        specimen.score(-30,600,0,-50,950,0);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,300,0, 920,50,0);
        specimen.score(-90,600,0,-110,950,0);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,300,0, 920,50,0);
        specimen.score(-140,600,0,-160,950,0);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,300,0, 900,50,0);
        specimen.score(-230,600,0,-250,950,0);

        specimen.specimenPark(1);
        //specimen.extendoPark();
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
