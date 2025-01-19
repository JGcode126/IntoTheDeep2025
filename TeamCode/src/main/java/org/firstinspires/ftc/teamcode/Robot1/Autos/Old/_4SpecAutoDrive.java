package org.firstinspires.ftc.teamcode.Robot1.Autos.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;

@Autonomous(name="4 Drive Spec ", group="Example")
@Config
@Disabled
public class _4SpecAutoDrive extends CuttleInitOpMode {
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

        specimenMethods.firstSpecimen(-730, 2.8);

        specimenMethods.sampleDriving(-700, -400, -750, -1350, -1050, -1350, -1050, -300);
        specimenMethods.sampleDriving(-1000, -1350,-1300, -1350,-1300, -300);
        specimenMethods.sampleDriving(-1300, -1300,-1500, -1300,-1450, -300);
        specimenMethods.drive(-1300, -700, Math.PI);
        //methods.drive(-500, -300, 0);

        specimenMethods.scoringSpecimenDrive(0, -4, 0,0,0);
        specimenMethods.scoringSpecimenDrive(0.25, -6,0,0,20);
        specimenMethods.scoringSpecimenDrive(0.75,-7,0, -10,50);

        specimenMethods.specimenTelePark();
    }

    public void mainLoop() {
        super.mainLoop();
        specimenMethods.telemetryData();
    }
}


