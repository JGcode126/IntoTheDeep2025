package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous(name = "bucket_5", group = "Example")
@Config
public class bucket extends CuttleInitOpModeRobot2 {
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

        super.main();
        liftPosController.setHome();
        extendoPosController.setHome();

        bucket.scoreFirstSample2(-280, -460,50, -380, -270,90,200);

        bucket.scoringBuckets2(-420, -270, 90, 5,-300, -470, 50, 200);
        bucket.scoringBuckets2(-420, -500, 90, 5,-300, -470, 50, 200);
        bucket.scoringBuckets2(-850, -300, 160, 1,-300, -370, 50, 200);

        bucket.middle(-1500, 0, 0, -1400, 500, 0,-300, -370, 50, 180, -1500, 0);

        bucket.park(-1500, 400, 180);
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
