package org.firstinspires.ftc.teamcode.Robot_V2.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous(name = "BUCKET", group = "Example")
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

        bucket.scoreFirstSample(-180, -780,60, -300, -780);

        bucket.scoringBuckets(-300, -780, 80, -200, -780, 60, 100);
        bucket.scoringBuckets(-300, -780, 109, -210, -780, 60, 130);
        bucket.scoringBuckets(-300, -780, 130, -220, -780, 60, 90);

        bucket.park(-1500, -500, 200);
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
