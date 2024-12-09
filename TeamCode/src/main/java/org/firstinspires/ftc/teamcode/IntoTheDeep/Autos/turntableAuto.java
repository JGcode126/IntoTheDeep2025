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
    public static int x = -450;
    public static int y = -250;

    public static int x2 = 0;
    public static int y2 = -650;
    public static double rotation = 45;

    public static double liftSpecimen = 3;

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

        methods.x = x;
        methods.y = y;
        methods.x2 = x2;
        methods.y2 = y2;
        methods.rotation = rotation;
        methods.liftSpecimen = liftSpecimen;
    }

    public void main(){
        super.main();

        methods.firstSpecimen();

        methods.ttSample();

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
