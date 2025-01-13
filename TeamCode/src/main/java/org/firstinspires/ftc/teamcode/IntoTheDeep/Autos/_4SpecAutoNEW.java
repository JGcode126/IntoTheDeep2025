package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.SpecimenAuto;

@Autonomous
@Config
public class _4SpecAutoNEW extends CuttleInitOpMode {
    public int loopCounter = 0;
    public static int distance = -850;

    public static int x = 0;
    public static int y = -360;
    public static int r = 0;

    public static int x1 = -100;
    public static int y1 = -600;
    public static int r1 = 0;

    public static int x2 = -300;
    public static int y2 = -400;
    public static int r2 = 45;

    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();
        setup.test = false;
        setup.side = "right";
        setup.color = "blue";
    }

    public void main(){
        super.main();
        loopCounter = 0;

        auto.startSpecimen(distance, 3, 0.6, 0.7);
        auto.allSamples();

        specimen.scoringSpecimenFancy(0, -4, 0,0,0);//1st specimen
        specimen.scoringSpecimenFancy(0.25, -6,0,0,20);//2nd specimen
        specimen.scoringSpecimenFancy(0.75,-7,0, 0,50);//3rd specimen
        specimen.scoringSpecimenFancy(1.5,-9,0, 0,70);//4th specimen

        specimen.specimenPark(1);
    }

    public void mainLoop() {
        super.mainLoop();
        loopCounter = specimen.yellowPark(loopCounter);
        specimenMethods.telemetryData();
    }
}
