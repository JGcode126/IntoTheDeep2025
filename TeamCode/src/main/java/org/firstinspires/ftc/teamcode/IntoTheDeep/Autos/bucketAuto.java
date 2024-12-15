package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
public class bucketAuto extends CuttleInitOpMode {
    public void onInit() {
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        methods.initRobot();

        methods.test = true;
        methods.side = "left";
    }

    public void main(){
        super.main();

        methods.firstSpecimen();
        //methods.specimen();

        methods.teleOpInit();
    }

    public void mainLoop() {
        super.mainLoop();
        methods.telemetryData();}
}
