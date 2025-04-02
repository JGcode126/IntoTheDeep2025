package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.SignColor.BLUESIGN;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.SignColor.REDSIGN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;

@Autonomous(name = "6 Bucket", group = "Example")
@Config
public class _6BucketAuto extends CuttleInitOpModeRobot2 {
    v2CuttleIntake.Color rejectColor;
    v2CuttleIntake.Color inColor;
    private ElapsedTime totalAutoTime = new ElapsedTime();

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


        totalAutoTime.reset();
        liftPosController.setHome();
        extendoPosController.setHome();

        bucket.scoreFirstSample6(-280, -440,60, 200, -280, -420, 60);

        bucket.scoringBuckets6(-460, -500, 90,5,-310, -490, 50, 600, 1.5,0.5);
        bucket.scoringBuckets6Pt2(-600, -450, 90, 3,-280, -450, 50, 00, 0,1);
        bucket.scoringBucketsLast2point0(2,-290, -440, 50, 1.5, -1200, -100,0);

        bucket.middle2(-1600, 50, 0, -620, -240, 50, RED, BLUE);
        bucket.middle3(-1600, 0, 0, -620, -410, 50, RED, BLUE);
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
