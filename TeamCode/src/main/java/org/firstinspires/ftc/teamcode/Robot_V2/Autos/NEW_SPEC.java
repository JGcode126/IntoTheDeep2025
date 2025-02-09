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
        specimen.sampleSweep(5, 500,600,-30,500,600,-110);

        specimen.sweepSetup(700,600,-30);
        specimen.sampleSweep(5, 700,600,-30,700,600,-110);

        specimen.scoreSetup();
        specimen.scoring(750,400,0, 750,0,0);
        specimen.score(0,600,0,-50,920,0);

        specimen.scoreSetup();
        specimen.scoring(750,400,0, 750,0,0);
        specimen.score(0,600,0,-50,920,0);

        specimen.scoreSetup();
        specimen.scoring(750,400,0, 750,0,0);
        specimen.score(0,600,0,-100,920,0);

        specimen.scoreSetup();
        specimen.scoring(750,400,0, 750,0,0);
        specimen.score(0,600,0,-50,920,0);
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
