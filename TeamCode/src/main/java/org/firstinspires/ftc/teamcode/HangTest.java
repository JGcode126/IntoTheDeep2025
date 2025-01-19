package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HangTest extends CuttleInitOpModeRobot2{

    public void onInit() {
        super.onInit();
    }
    public void main() {super.main();}
    public void mainLoop() {
        super.mainLoop();

        if(gamepad1.y){
            hang.servoUp();
        }
        if(gamepad1.a){
            hang.servoDown();
        }
        telemetry.update();
    }
}
