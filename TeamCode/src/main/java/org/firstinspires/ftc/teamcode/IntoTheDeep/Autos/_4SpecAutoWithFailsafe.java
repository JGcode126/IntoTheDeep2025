package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous(name="FAILSAFE_4_Spec", group="Example")
@Config
public class _4SpecAutoWithFailsafe extends CuttleInitOpMode {
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

        specimenMethods.firstSpecimen(-850, 3);

        specimenMethods.ttSample();

        specimenMethods.scoring3Failsafe();

        specimenMethods.specimenTelePark();

    }

    public void mainLoop() {
        super.mainLoop();
        specimenMethods.telemetryData();
    }
}


