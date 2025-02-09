package org.firstinspires.ftc.teamcode.Robot_V2.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@TeleOp
@Config
public class Tester_opmode extends CuttleInitOpModeRobot2 {
    ElapsedTime timer;
    public void onInit() {
        super.onInit();
        timer = new ElapsedTime();
    }
    public void main() {
        super.main();
        timer.reset();
    }
    public void mainLoop() {
        super.mainLoop();

        timeHang.hangDown(timer.seconds());

        telemetry.addData("Color:", intake.getColor());
        telemetry.update();
    }
}
