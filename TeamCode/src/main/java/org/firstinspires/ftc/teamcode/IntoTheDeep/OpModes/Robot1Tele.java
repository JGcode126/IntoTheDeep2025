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

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@TeleOp
@Config
public class Robot1Tele extends CuttleInitOpMode{
    public static double SlidePos = 0;
    public void onInit() {
        super.onInit();

        //extendoPosController.enable();
        //extendoMotor.enablePositionPID(true);
        //extendoPosController.setHome();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void main() {
        super.main();

    }

    public void mainLoop() {
        super.mainLoop();


        //extendo.setSlidePosition(SlidePos);
        //intake.intakePos(0.5);
        intake.initPos();

        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("slides", extendoPosController.getPosition());
        telemetry.update();
    }
}
