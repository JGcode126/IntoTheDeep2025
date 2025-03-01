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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();



        out = outColor;
        in = inColor;
    }

    public void main(){
        super.main();
        totalAutoTime.reset();
        liftPosController.setHome();
        extendoPosController.setHome();

        bucket.scoreFirstSample2(-280, -440,50, -370, -270,90,200);

        bucket.scoringBuckets2(-420, -270, 90, 5,-300, -470, 50, 200);
        bucket.scoringBuckets2(-420, -515, 90, 5,-300, -470, 50, 200);
        bucket.scoringBucketsLast(-850, -300, 160, 1,-270, -370, 50, 200);

        bucket.middle(-1300, 0, 0, -1400, 500, 0,-280, -350, 50, 180, -1500, 0, out, in);

        bucket.park(-1400, 350, 180);
    }

    public void mainLoop() {
        super.mainLoop();

        if (intake.getColor() == BLUE && totalAutoTime.seconds() > 17){
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
        }
        setup.telemetryData();
    }
}
