package org.firstinspires.ftc.teamcode.Robot_V2.Autos.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

//@Autonomous(name = "1+3_bucket", group = "Example")
@Config
@Disabled
public class specimen_bucket extends CuttleInitOpModeRobot2 {
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

        specimen.scoreSpecBucket(-500,1500,90, 0.6,0.4);

        bucket.scoringBuckets(-420, -300, 90, 5,-300, -500, 50, 90, -300, -500);
        bucket.scoringBuckets(-420, -500, 90, 5,-300, -500, 50, 120, -300, -500);
        bucket.scoringBuckets(-600, -350, 140, 3,-300, -500, 50, 90, -300, -500);
        //bucket.messUpMiddle(-1300, 0, 0, -1300, 500, 0);

        bucket.park(-1200, 0, 180);
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
