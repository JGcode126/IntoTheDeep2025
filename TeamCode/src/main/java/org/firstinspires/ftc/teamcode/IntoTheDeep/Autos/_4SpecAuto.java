package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.AutoSequence;

@Autonomous(name="4 Spec", group="Example")
@Config
public class _4SpecAuto extends CuttleInitOpMode {
    public static int distance = -650;
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();

        setup.test = false;
        setup.side = "right";
        methods.color = "blue";
    }

    public void main(){
        super.main();

        methods.firstSpecimen(-730, 2.8);

        methods.sampleTT_Test(-685,-620,90);
        methods.sampleTT_Test(-945,-660,90);
        methods.sampleTT_Test(-1250,-710,100);

        methods.scoring3();
        methods.specimenTelePark();
    }

    public void mainLoop() {
        super.mainLoop();
        methods.telemetryData();
    }
}


