package org.firstinspires.ftc.teamcode.Robot1.Autos;

import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.Color.YELLOW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;

@Autonomous(name="FAILSAFE_4_Spec", group="Example")
@Config
@Disabled
public class _4SpecAutoWithFailsafe extends CuttleInitOpMode {
    public static int distance = -650;
    public int loopCounter = 0;
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
        loopCounter = 0;
        specimenMethods.firstSpecimen(-850, 3);

        specimenMethods.ttSample();

        specimenMethods.scoring3();

        specimenMethods.specimenTelePark();

    }

    public void mainLoop() {
        super.mainLoop();
        loopCounter += 1;
        if (loopCounter == 25){
            loopCounter= 0;
            if (intake.getColor() == YELLOW) {
                queue.clear();
                loopCounter = 0;
                specimenMethods.specimenTelePark();
            }
        }
        specimenMethods.telemetryData();
    }
}


