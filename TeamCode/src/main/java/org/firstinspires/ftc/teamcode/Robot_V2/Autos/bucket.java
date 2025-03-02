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
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.Task;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;

@Autonomous(name = "bucket_5", group = "Example")
@Config
public class bucket extends CuttleInitOpModeRobot2 {
    v2CuttleIntake.Color out;
    v2CuttleIntake.Color in;
    private ElapsedTime failSafeTimer = new ElapsedTime();
    private ElapsedTime totalAutoTime = new ElapsedTime();
    private ElapsedTime spitTime = new ElapsedTime();
    public void onInit() {
        super.onInit();
        bucketAuto = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();

        /*try {
            v2CuttleIntake.SignColor signColor = intake.getSignColor();

            if (signColor == BLUESIGN) {
                in = RED;
                out = BLUE;
            } else if (signColor == REDSIGN) {
                in = BLUE;
                out = RED;
            }

            telemetry.addData("Sign Color", signColor);
        } catch (Exception e) {
            telemetry.addLine("Error retrieving sign color: " + e.getMessage());
        }

        telemetry.addData("IN Color", inColor);
        telemetry.addData("OUT Color", outColor);
        telemetry.update();*/
    }

    public void main(){
        super.main();
        totalAutoTime.reset();
        liftPosController.setHome();
        extendoPosController.setHome();

        bucket.scoreFirstSample2(-280, -460,60, 200);

        bucket.scoringBuckets2(-350, -280, 90, 6,-300, -470, 50, 200, 0);
        bucket.scoringBuckets2(-420, -515, 90, 5,-300, -470, 50, 200, 0);
        bucket.scoringBucketsLast(-920, -250, 170, 2,-320, -350, 50, 200);

        bucket.middle2(-1300, 0, 0, -1400, 500, 0,-300, -400, 50, out, in);

        bucket.park(-1500, 0, 180,-1400, 350, 180);
    }

    public void mainLoop() {
        super.mainLoop();

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
