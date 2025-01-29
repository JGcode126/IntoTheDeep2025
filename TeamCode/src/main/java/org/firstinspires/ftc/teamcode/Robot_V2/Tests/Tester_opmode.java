package org.firstinspires.ftc.teamcode.Robot_V2.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@TeleOp
@Config
public class Tester_opmode extends CuttleInitOpModeRobot2 {
    public void onInit() {
        super.onInit();

    }
    public void main() {
        super.main();
    }
    public void mainLoop() {
        super.mainLoop();
        outake.backIntakePos();
        liftPosition = 1.6;


        telemetry.addData("Color:", intake.getColor());
        telemetry.update();
    }
}
