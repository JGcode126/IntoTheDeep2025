package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.SignColor.BLUESIGN;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.SignColor.REDSIGN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.Task;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;

@Autonomous(name = "bucket_6", group = "Example")
@Config
public class _6BucketAuto extends CuttleInitOpModeRobot2 {
    private ElapsedTime totalAutoTime = new ElapsedTime();
    public void onInit() {
        super.onInit();
        bucketAuto = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();
    }

    public void main(){
        super.main();

        totalAutoTime.reset();
        liftPosController.setHome();
        extendoPosController.setHome();

        bucket.scoreFirstSampleFor6(-280, -460,60, 200, 1.5,-350, -260, 90);

        bucket.scoringBucketsFor6(5,-300, -470, 50, 250, 0.5,1.6, -420,-525,90);
        bucket.scoringBucketsFor6(5,-300, -470, 50, 250, 0,1.6, -920,-250,170);
        bucket.scoringBucketsLastFor6(1.5,-260, -330, 50, 400,3);

        bucket.middleFor6(-1350, 0, 0, -520, -340, 50, 0.8, RED, BLUE);
        bucket.middle3For6(-1600, 0, 0, -530, -390, 50, 0.8, RED, BLUE);

        bucket.parkFor6(-1500, 0, 180,-1400, 350, 180);
    }

    public void mainLoop() {
        super.mainLoop();

        /*if(totalAutoTime.seconds() >= 28 && intake.getColor() == null){
            queue.clear();
            queue.addTask(new CustomTask(() -> {
                intake.out();
                intake.armUp();
                extendoPosition = 0;
                liftPosition = 0;
                return true;
            }));

            queue.addTask(new DelayTask(200));

            queue.addTask(new PointTask(new Waypoint(new Pose(-1500, 0, Math.toRadians(180)), 0.9,0.5,100, false), ptpController));

            queue.addTask(new CustomTask(() -> {
                intake.turntableMiddle();
                //outake.readyPos();
                //hang.parkHeight();
                intake.armUp();
                extendoPosition = 0;
                liftPosition = 0;
                outake.parkPos();

                return true;
            }));

            queue.addTask(new PointTask(new Waypoint(new Pose(-1400, 350, Math.toRadians(180)), 0.9,0.5,100, false), ptpController));

            queue.addTask(new CustomTask(() -> {
                dt.drive(-0.2,0,0);
                return true;
            }));

            queue.addTask(new DelayTask(60000));
        }*/

        setup.telemetryData();
    }
}
