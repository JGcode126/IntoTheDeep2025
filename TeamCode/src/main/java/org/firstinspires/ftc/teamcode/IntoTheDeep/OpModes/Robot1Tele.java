package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.Task;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@TeleOp
public class Robot1Tele extends CuttleInitOpMode{
    public void onInit() {
        super.onInit();

        extendoPosController.enable();
        extendoMotor.enablePositionPID(true);
        extendoPosController.setHome();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void main() {
        super.main();
        queue.addTask(new CustomTask(()->{
            extendoMotor.setPower(1);
            extendoMotor.setPosition(7);
            return true;
        }));
        queue.addTask(new DelayTask(1000));
        queue.addTask(new CustomTask(()->{
            //extendoMotor.setPosition(0);
            return true;
        }));

    }

    public void mainLoop()
    {
        super.mainLoop();


        //extendoMotor.setPosition(100);


        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.update();
    }
}
