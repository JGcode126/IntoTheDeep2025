package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous(name="EXTRA SAMPLE 4 SPEC", group="Example")
@Config
@Disabled
public class _4_Extra_SpecAuto extends CuttleInitOpMode {
    public static int distance = -650;
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();

        setup.test = false;
        setup.side = "right";
        specimenMethods.color = "blue";
    }

    public void main(){
        super.main();

        specimenMethods.firstSpecimenBitFaster(-800, 3);

        specimenMethods.ttSample3Different();

        specimenMethods.scoring3Different();

        specimenMethods.specimenTeleParkFaster();

    }

    public void mainLoop() {
        super.mainLoop();
        specimenMethods.telemetryData();
    }
}


