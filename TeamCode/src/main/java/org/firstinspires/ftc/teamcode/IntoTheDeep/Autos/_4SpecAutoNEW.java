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
    public static double speed1 = 0.4;
    public static double speed2 = 0.6;

    public static int x2 = -1000;
    public static int y1 = -630;
    public static int r2 = 50;
    public static int x1 = -1180;
    public static double extF = 4;
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
        //r2 used to be 70, turn = -0.12
        //x1 = -580, y1 = -720, r1 = 90, x2 = -850, y2 = 400, r2 = 80
        specimen.ttSample(-580, -720,90,-850,-600, 50, -0.25);//first sample
        //x1 = 920, x2 = -850, y2 = -400, r2 = 70
        specimen.ttSample(-870, -720,90,-1000,-550, 50, -0.25);//second sample
        specimen.ttSample(2,4, -1180, -640,110,-850,-400, 50, -0.25);//third sample
        //auto.allSamples();
        //auto.scoringSpec3Fancy();
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
