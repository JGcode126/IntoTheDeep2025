package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Pose;


@TeleOp(name="CuttleCoords", group="Example ")
public class AutoCoords extends CuttleInitOpModeMTI {

    public void onInit() {
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }
    public void main() {
        super.main();

        queue.addTask(new PointTask(new Waypoint(new Pose(-1600,450,-Math.PI/2)), ptpController));
        queue.addTask(new DelayTask(1000));
        queue.addTask(new PointTask(new Waypoint(new Pose(-1600,-300,Math.PI/2)), ptpController));
        queue.addTask(new DelayTask(1000));
        queue.addTask(new PointTask(new Waypoint(new Pose(-1600,450,-Math.PI/2)), ptpController));
        queue.addTask(new DelayTask(1000));
        queue.addTask(new PointTask(new Waypoint(new Pose(0,0,0)), ptpController));



    }

    public void mainLoop()
    {
        super.mainLoop();

        queue.addTask(new PointTask(new Waypoint(new Pose(0,0,0)), ptpController));
        //chassis.setVec(new Pose(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x));

        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.update();
    }





}
