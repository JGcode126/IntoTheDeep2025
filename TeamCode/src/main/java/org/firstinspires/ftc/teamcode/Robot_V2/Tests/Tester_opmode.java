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
        hang.teleHeight();
    }
    public void mainLoop() {
        super.mainLoop();
        if (gamepad2.share){
            hang.hangHeight();
        }


        telemetry.addData("Color:", intake.getColor());
        telemetry.update();
    }
}
