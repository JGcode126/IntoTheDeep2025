package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous(name = "bucket_4", group = "Example")
@Config
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

        bucket.scoreFirstSample(-280, -440,50, -380, -250,90);

        bucket.scoringBuckets(-420, -250, 90, 5,-300, -450, 50, 90, -300, -500);
        bucket.scoringBuckets(-420, -500, 90, 5,-300, -450, 50, 160, -600, -300);
        bucket.scoringBuckets(-850, -300, 160, 1,-300, -350, 50, 180, -1400, 0);

        bucket.park(-1400, 400, 180);
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
