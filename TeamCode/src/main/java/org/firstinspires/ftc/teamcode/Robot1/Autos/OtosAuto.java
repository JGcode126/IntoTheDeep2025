package org.firstinspires.ftc.teamcode.Robot1.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;

@Autonomous(name="otos", group="Example")
@Config
@Disabled
public class OtosAuto extends CuttleInitOpMode {
    public static int x = 0;
    public static int y = 0;
    public static int r = 0;
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();
        //setup.runTest();

        setup.test = false;
        setup.side = "right";
        specimenMethods.color = "blue";
    }

    public void main(){
        super.main();
        /*specimenMethods.firstSpecimen(-600, 3);
        specimenMethods.ttSample();
        specimenMethods.scoring3();
        specimenMethods.specimenTelePark();*/

        specimenMethods.addWaypointTaskDefault(new Pose(x, y, Math.toRadians(r)));
    }

    public void mainLoop() {
        super.mainLoop();
        setup.allLocationData();
    }
}


