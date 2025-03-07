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

@Autonomous(name = "bucket_5", group = "Example")
@Config
public class bucket extends CuttleInitOpModeRobot2 {
    private int loopCounter = 0;
    v2CuttleIntake.Color rejectColor;
    v2CuttleIntake.Color inColor;
    private ElapsedTime failSafeTimer = new ElapsedTime();
    private ElapsedTime totalAutoTime = new ElapsedTime();
    private ElapsedTime spitTime = new ElapsedTime();
    public void onInit() {
        super.onInit();
        bucketAuto = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();
    }

    public void main(){
        super.main();
        if (intake.getSignColor() == BLUESIGN){
            rejectColor = BLUE;
            inColor = RED;
        }
        if (intake.getSignColor() == REDSIGN){
            rejectColor = RED;
            inColor = BLUE;
        }
        loopCounter = 0;

        totalAutoTime.reset();
        liftPosController.setHome();
        extendoPosController.setHome();

        bucket.scoreFirstSample2(-280, -460,60, 200);

        bucket.scoringBuckets2(-350, -260, 90, 5,-300, -470, 50, 250, 0.5,1.6);
        bucket.scoringBuckets2(-420, -525, 90, 5,-300, -470, 50, 250, 0,1.6);
        bucket.scoringBucketsLast(-920, -250, 170, 2,-260, -330, 50, 400,3);

        bucket.middle2(-1300, 0, 0, -1400, 500, 0,-400, -420, 50, RED, BLUE);
        bucket.middle3(-1300, 0, 0, -1400, 500, 0,-400, -420, 50, RED, BLUE);

        bucket.park(-1500, 0, 180,-1400, 350, 180);
    }

    public void mainLoop() {
        super.mainLoop();

        /*if(totalAutoTime.seconds() >= 28){
            queue.clear();
            //bucket.park(-1500, 0, 180,-1400, 350, 180);

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

        /*if (intake.getColor() == BLUE && totalAutoTime.seconds() > 17){
            if (failSafeTimer.seconds() > 0.5) {
                queue.pause();
                spitTime.reset();
                Task currentTask = queue.getTask();

                currentTask.kill();
                queue.getTasks().addFirst(currentTask);
                queue.getTasks().addFirst(new CustomTask(() -> {
                    //intake.in();
                    intake.turntable.setPosition(0.2);
                    extendoPosition = 1.8;
                    return true;
                }));

                queue.getTasks().addFirst(new DelayTask(2000));
                queue.getTasks().addFirst(new CustomTask(() -> {
                    intake.out();
                    extendoPosition = 0.5;
                    return intake.getColor() != BLUE && spitTime.seconds() >= 5;
                }));

                queue.unpause();
                failSafeTimer.reset();
            }
        }else{
            failSafeTimer.reset();
        }*/
        setup.telemetryData();
    }
}
