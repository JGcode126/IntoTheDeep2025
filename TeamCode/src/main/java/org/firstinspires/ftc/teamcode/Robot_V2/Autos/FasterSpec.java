package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous(name = "specimen_5+1", group = "Example")
@Config
public class FasterSpec extends CuttleInitOpModeRobot2 {
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

        specimen.scoreOther(-150,900,0, 0.6,0.8);

        specimen.sweepSetup(500,600,-30);
        specimen.sampleSweep(4.9, 500,600,-30, 300,0.5,0.8);

        specimen.sweepSetup(770,670,-40);
        specimen.sampleSweep(5.2, 770,670,-40,300,0.5,0.8);

        specimen.sweepSetup(970,870,-60);
        specimen.sampleSweep(4.8, 970,870,-60,200,0.5,0.8);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 920,50,0,0.6);
        specimen.score(-70,600,0,-70,950,0,0.8,0.8);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 920,50,0,0.6);
        specimen.score(-90,600,0,-130,950,0,0.8,0.8);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 920,50,0,0.6);
        specimen.score(-140,600,0,-180,950,0,0.8,0.8);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 900,50,0,0.6);
        specimen.score(-230,600,0,-270,950,0,0.8,0.8);

        specimen.scoreSetup();
        specimen.intakeOffWall(900,400,0, 900,50,0,0.6);
        specimen.scoreBucket(-1000,600,0,-270,950,0,1,1);


        //specimen.specimenPark(1);
        //specimen.extendoPark();
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
