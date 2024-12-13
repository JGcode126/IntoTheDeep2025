package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.TaskList;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
public class turntableAuto extends CuttleInitOpMode {
    //For testing
    //public static int x = -1055;
    //y1 = -425, y2 = -650, y3 = -400
    public static int y1 = -425;
    //public static int r = 90;
    //2nd sample values = x = -1055, y = -580, r = 90
    //fist sample values x = -810, y = -590, r = 90

    //public static int x2 = -900;
    public static int y2 = -650;
    public static int y3 = -400;
    public static int r2 = 50;
    //x2 = -900, y2 = -400, r2 = 50
    //-----------

    private ElapsedTime timer;
    public void onInit() {
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        timer = new ElapsedTime();
        timer.reset();

        methods.initRobot();

        methods.test = true;
        methods.side = "right";
        methods.color = "blue";
    }

    public void main(){
        super.main();

        methods.firstSpecimen();
        //methods.ttSample();
        methods.fistSampleTT();
        methods.secondSampleTT();
        methods.thridSampleTT();
        //methods.testSampleTT(-1260, -610,100,-900,-400,50);

        //y1 = -425, y2 = -650, y3 = -400
        methods.scoringTest(300,y1, y2, y3);
        methods.scoringSpecimen(300);
        methods.scoringSpecimen(600);
        methods.scoringSpecimen(900);
        //methods.scoringSpecimen();

        methods.park();
    }

    public void mainLoop() {
        super.mainLoop();
        //otosLocalizer.update();
        methods.telemetryData();
    }

    void intakeColor(){
        TaskList suck = new TaskList();
        suck.addTask(new CustomTask(()->{
            intake.intakePos(intake.turntableInitPos);
            intake.in();
            intake.turntableRight();
            return true;
        }));
        queue.addTask(suck);
    }
}
