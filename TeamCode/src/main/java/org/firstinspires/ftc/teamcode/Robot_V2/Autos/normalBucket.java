package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous(name = "bucket_4", group = "Example")
@Config
@Disabled
public class normalBucket extends CuttleInitOpModeRobot2 {
    public void onInit() {
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();

        setup.test = false;
        setup.side = "right";
        setup.color = "blue";
    }

    public void main(){
        super.main();
        liftPosController.setHome();
        extendoPosController.setHome();

        liftPosController.setHome();
        extendoPosController.setHome();

        bucket.scoreFirstSample2(-280, -460,60, 200);

        bucket.scoringBuckets2(-350, -260, 90, 5,-300, -470, 50, 250, 0.5,1.6);
        bucket.scoringBuckets2(-420, -525, 90, 5,-300, -470, 50, 250, 0,1.6);
        bucket.scoringBucketsLast(-920, -250, 170, 1.5,-260, -330, 50, 400,3);

        bucket.park(-1500, 0, 180,-1400, 350, 180);
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
