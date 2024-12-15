package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous(name="4 Spec Auto", group="Example")
public class _4SpecAuto extends CuttleInitOpMode {
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        methods.initRobot();

        methods.test = false;
        methods.side = "right";
    }

    public void main(){
        super.main();

        methods.firstSpecimen();
        methods.ttSample();
        methods.scoring3();
        methods.park();
    }

    public void mainLoop() {
        super.mainLoop();
        methods.telemetryData();
    }
}
