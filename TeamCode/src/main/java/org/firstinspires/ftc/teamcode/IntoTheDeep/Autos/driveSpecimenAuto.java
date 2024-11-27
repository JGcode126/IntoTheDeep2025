package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
public class driveSpecimenAuto extends CuttleInitOpMode {
    public static int x = -300;
    public static int y = -400;

    public static int x2 = 150;
    public static int y2 = -650;
    public static double rotation = 45;
    public void onInit() {
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        methods.initRobot();

        methods.test = true;
        methods.side = "right";
        methods.color = "blue";

        methods.x = x;
        methods.y = y;
        methods.x2 = x2;
        methods.y2 = y2;
        methods.rotation = rotation;
    }

    public void main(){
        super.main();

        methods.firstSpecimen();
        methods.sample();

        methods.scoringSpecimen();
        methods.scoringSpecimen();
        methods.scoringSpecimen();
        methods.scoringSpecimen();

        methods.teleOpInit();
    }

    public void mainLoop() {
        super.mainLoop();
        //otosLocalizer.update();
        methods.telemetryData();
    }
}
