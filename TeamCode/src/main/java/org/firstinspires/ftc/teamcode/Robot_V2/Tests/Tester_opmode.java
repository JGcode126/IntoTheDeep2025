package org.firstinspires.ftc.teamcode.Robot_V2.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Pose;

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
        //queue.addTask(new PointTask(new Waypoint(new Pose(0, 1000, 0)), ptpController));

    }
    public void mainLoop() {
        super.mainLoop();

        timeHang.hangDown(4);


        telemetry.addData("Color:", intake.getColor());
        telemetry.update();
    }
}
