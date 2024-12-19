package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
@Disabled
public class OLDdriveSpecimenAuto extends CuttleInitOpMode {
    private ElapsedTime timer;
    public void onInit() {
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        timer = new ElapsedTime();
        timer.reset();

        specimenMethods.initRobot();

        specimenMethods.test = true;
        specimenMethods.side = "right";
        specimenMethods.color = "blue";
    }

    public void main(){
        super.main();

        specimenMethods.firstSpecimen();

        specimenMethods.sample();

        //methods.scoringSpecimen();
        //methods.scoringSpecimen();
        //methods.scoringSpecimen();
        //methods.scoringSpecimen();

        specimenMethods.specimenPark();
    }

    public void mainLoop() {
        super.mainLoop();
        //otosLocalizer.update();
        specimenMethods.telemetryData();
    }
}
