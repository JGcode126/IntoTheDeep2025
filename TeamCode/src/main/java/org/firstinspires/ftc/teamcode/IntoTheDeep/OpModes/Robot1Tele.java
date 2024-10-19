package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.SetMotorPositionTask;
import com.roboctopi.cuttlefish.queue.Task;
import com.roboctopi.cuttlefishftcbridge.tasks.MotorPositionTask;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@TeleOp
@Config
public class Robot1Tele extends CuttleInitOpMode{
    double targetPos = 0;
    public void onInit() {
        super.onInit();

        intake.initPos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void main() {
        super.main();

    }

    public void mainLoop() {
        super.mainLoop();

        if(gamepad1.right_bumper){
            targetPos += 0.3;
        }
        if (gamepad1.left_bumper){
            targetPos -= 0.3;
        }

        position = targetPos;

        intake.intakeMachine(gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left);


        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("slides", extendoPosController.getPosition());
        telemetry.addData("color", intake.getColor());
        telemetry.addData("intake state", intake.currentState);
        telemetry.update();
    }


}
