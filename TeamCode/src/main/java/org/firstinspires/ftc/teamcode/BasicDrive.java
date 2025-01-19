package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.CuttleInitOpModeRobot2;
import org.firstinspires.ftc.teamcode.Testing.CuttleInitOpModeMTI;


@TeleOp
public class BasicDrive extends CuttleInitOpModeRobot2 {
    public void onInit() {
        super.onInit();
    }
    public void main() {
        super.main();
    }
    public void mainLoop()
    {
        super.mainLoop();
        hang.servoUp();
        chassis.setVec(new Pose(gamepad1.right_stick_x*0.4,-gamepad1.right_stick_y*0.4,-gamepad1.left_stick_x*0.4));
    }
}