package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Testing.CuttleInitOpModeMTI;


@TeleOp
//@Disabled
public class DemoMode extends CuttleInitOpModeMTI {
    public void onInit() {
        super.onInit();
    }
    public void main() {
        super.main();
    }
    public void mainLoop()
    {
        super.mainLoop();
        chassis.setVec(new Pose(gamepad1.left_stick_x*0.4,-gamepad1.left_stick_y*0.4,-gamepad1.right_stick_x*0.4));
    }
}