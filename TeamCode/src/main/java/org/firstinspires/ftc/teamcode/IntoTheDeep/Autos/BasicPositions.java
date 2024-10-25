package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
public class BasicPositions extends CuttleInitOpMode {
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftPosition = 0;
        extendoPosition = 0;
        intake.initPos();
    }

    public void main(){
        super.main();
        queue.addTask(new CustomTask(()->{
            encoderLocalizer.reset();
            return true;
        }));
        queue.addTask(new CustomTask(()->{
            liftPosition = 5;
            return true;
        }));
        queue.addTask(new PointTask(new Waypoint(new Pose(0,-550,0)), ptpController));
        //queue.addTask(new DelayTask(1000));

        //queue.addTask(new PointTask(new Waypoint(new Pose(0,0,0)), ptpController));
    }

    public void mainLoop(){
        super.mainLoop();

        encoderLocalizer.update();
        System.out.println(encoderLocalizer.getPos());
        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        //telemetry.addData("otos r", pos.h);
        //telemetry.addData("otos x", pos.x);
        //telemetry.addData("otos y", pos.y);
        telemetry.update();
    }
}
