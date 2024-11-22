package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
public class PIDTesting extends CuttleInitOpMode {

    private ElapsedTime timer = new ElapsedTime();

    public void onInit(){
        super.onInit();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        otosLocalizer.reset();

        liftPosition = 0;
        extendoPosition = 0;

        intake.initPos();
        liftPosController.setHome();

        outake.closeClaw();
        outake.autoPos();

    }
    public void main() {
        super.main();
        queue.addTask(new PointTask(new Waypoint(new Pose(0, 2000, 0)), ptpController));





    }
    public void mainLoop(){
        super.mainLoop();

        queue.addTask(new PointTask(new Waypoint(new Pose(0, 2000, 0)), ptpController));

        otosLocalizer.update();
        System.out.println(otosLocalizer.getPos());
        telemetry.addData("Intake Color", intake.getColor());
        telemetry.addData("Cuttle X:",otosLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",otosLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",otosLocalizer.getPos().getR());
        telemetry.addData("otos r", pos.h);
        telemetry.addData("otos x", pos.x);
        telemetry.addData("otos y", pos.y);
        telemetry.update();
    }
}
