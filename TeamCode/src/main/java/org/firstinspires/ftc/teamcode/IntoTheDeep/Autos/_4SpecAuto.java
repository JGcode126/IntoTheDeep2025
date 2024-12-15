package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.AutoSequence;

@Autonomous(name="4 Spec Auto", group="Example")
public class _4SpecAuto extends CuttleInitOpMode {
    private AutoSequence auto;
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();

        setup.test = false;
        setup.side = "right";

    }

    public void main(){
        super.main();

        auto.firstSpecimen();
        auto.ttSample();
        auto.scoring3();
        auto.specimenPark();
    }

    public void mainLoop() {
        super.mainLoop();
        setup.telemetryData();
    }
}
