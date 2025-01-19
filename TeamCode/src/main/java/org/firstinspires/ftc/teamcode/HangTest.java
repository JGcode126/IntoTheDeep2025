package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HangTest extends CuttleInitOpModeRobot2 {
    public void onInit() {
        super.onInit();
        //intake.initPos();
    }
    public void main() {
        super.main();


    }
    public void mainLoop() {
        super.mainLoop();
        hang.servoUp();

        telemetry.update();
    }
}
