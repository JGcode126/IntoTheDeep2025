package org.firstinspires.ftc.teamcode.Robot_V2.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@TeleOp
public class HangTest extends CuttleInitOpModeRobot2 {
    public void onInit() {
        super.onInit();
        hang.initHang();
    }
    public void main() {
        super.main();
    }
    public void mainLoop() {
        super.mainLoop();
        //hang.servoUp();
        //hang.servoDown();

        telemetry.update();
    }
}
