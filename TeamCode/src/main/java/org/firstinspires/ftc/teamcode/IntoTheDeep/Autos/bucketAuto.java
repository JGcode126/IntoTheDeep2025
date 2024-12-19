package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
public class bucketAuto extends CuttleInitOpMode {
    public void onInit() {
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bucketMethods.initRobot();

        specimenMethods.test = true;
        specimenMethods.side = "left";
    }

    public void main(){
        super.main();

        bucketMethods.scoreFirstSample(-870);
        bucketMethods.scoring(-870, -1130,0);
        bucketMethods.scoring(-1130, -1200,0);
        bucketMethods.scoring(-1200, -400,20);
        bucketMethods.park();

    }

    public void mainLoop() {
        super.mainLoop();
        specimenMethods.telemetryData();}
}
