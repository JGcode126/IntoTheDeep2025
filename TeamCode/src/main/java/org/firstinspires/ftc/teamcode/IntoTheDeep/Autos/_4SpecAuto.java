package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.AutoSequence;

@Autonomous(name="4 Spec Auto", group="Example")
@Config
public class _4SpecAuto extends CuttleInitOpMode {
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

        methods.firstSpecimen();
        methods.ttSample();
        methods.scoring3();
        methods.specimenTelePark();
    }

    public void mainLoop() {
        super.mainLoop();
        methods.telemetryData();
    }
}


