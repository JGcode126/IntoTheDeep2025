package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous(name="4 Drive Spec ", group="Example")
@Config
public class _4SpecAutoDrive extends CuttleInitOpMode {
    public static int distance = -650;
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setup.initRobot();

        setup.test = false;
        setup.side = "right";
        methods.color = "blue";
    }

    public void main(){
        super.main();

        methods.firstSpecimen(-730, 2.8);

        methods.sampleDriving(-700, -400, -750, -1350, -1050, -1350, -1050, -300);
        methods.sampleDriving(-1000, -1350,-1300, -1350,-1300, -300);
        methods.sampleDriving(-1300, -1300,-1500, -1300,-1450, -300);
        methods.drive(-1300, -700, Math.PI);
        //methods.drive(-500, -300, 0);

        methods.scoringSpecimenDrive(0, -4, 0,0,0);
        methods.scoringSpecimenDrive(0.25, -6,0,0,20);
        methods.scoringSpecimenDrive(0.75,-7,0, -10,50);

        methods.specimenTelePark();
    }

    public void mainLoop() {
        super.mainLoop();
        methods.telemetryData();
    }
}


