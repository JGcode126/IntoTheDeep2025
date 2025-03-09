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

@Autonomous(name = "BLUE_bucket_5", group = "Example")
@Config
public class BLUE_bucket extends CuttleInitOpModeRobot2 {
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

        bucket.scoreFirstSample2(-280, -460,60, 200);

        bucket.scoringBuckets2(-350, -240, 90, 5,-300, -470, 50, 250, 0.5,2);
        bucket.scoringBuckets2(-420, -525, 90, 5,-300, -470, 50, 250, 0,2);
        bucket.scoringBucketsLast(-940, -250, 170, 1.9,-240, -320, 50, 475,3);

        bucket.middle2(-1320, 50, 0, -1450, 500, 0,-520, -300, 50, BLUE, RED);
        //bucket.middle3(-1600, 0, 0, -1600, 500, 0,-530, -390, 50, RED, BLUE);

        bucket.park(-1500, 0, 180,-1400, 350, 180);
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
