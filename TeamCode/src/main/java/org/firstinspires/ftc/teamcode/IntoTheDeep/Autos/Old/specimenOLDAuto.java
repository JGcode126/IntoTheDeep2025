package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
@Disabled
public class specimenOLDAuto extends CuttleInitOpMode {
    public void onInit() {
        super.onInit();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        specimenMethods.initRobot();


        specimenMethods.test = false;
        specimenMethods.side = "right";
        specimenMethods.color = "blue";
    }

    public void main() {
        super.main();

        specimenMethods.sample();
        specimenMethods.firstSpecimen();
        specimenMethods.firstSpecimen();
        specimenMethods.secondSample();
        specimenMethods.thridSample();

        //methods.scoringSpecimen();
        //methods.scoringSpecimen();
        //methods.scoringSpecimen();

        specimenMethods.teleOpInit();
    }

    public void mainLoop() {
        super.mainLoop();
        specimenMethods.telemetryData();
    }

    private enum State {
        TO_SAMPLE,
        INTAKING_SAMPLE,
        SPIT_OUT_SAMPLE
    }
}
