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
    //------------For testing----------
    //public static int x = -100;
    //public static int y1 = -425;
    //public static int r = 90;

    //public static int x2 = -900;
    //public static int y2 = -750;
    //public static double lift = 5.1;
    public static int x1 = 0;
    public static int y1 = 0;
    public static int x2 = 0;
    public static int y2 = 0;
    public static int x3 = 0;
    public static int y3 = 0;
    public static int x4 = 0;
    public static int y4 = 0;

    public static int r1 = 0;
    public static int r2 = 0;
    public static int r3 = 0;
    public static int r4 = 0;

    public static double extOffset2 = 0.25;
    public static double extOffset3 = 0.5;
    public static double extOffset4 = 0.75;

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

        //methods.highChamberPos = lift;
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
        methods.scoringSpecimen(0, -4, 0,0,0);
        methods.scoringSpecimen(0.25, -6,0,0,20);
        methods.scoringSpecimen(0.75,-7,0, -10,50);
        methods.scoringSpecimen(3,-11,50,-20,90);

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
