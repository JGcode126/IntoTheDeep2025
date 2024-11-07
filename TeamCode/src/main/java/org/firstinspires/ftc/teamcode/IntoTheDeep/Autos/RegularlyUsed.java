package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
public class RegularlyUsed extends CuttleInitOpMode {
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        encoderLocalizer.reset();

        liftPosition = 0;
        extendoPosition = 0;

        intake.initPos();
        liftPosController.setHome();

        outake.closeClaw();
        outake.autoPos();
    }
    public void main() {
        super.main();

        queue.addTask(new CustomTask(()->{
            scoringSpecimen();
            return true;
        }));
    }

    void scoringSpecimen(){
        TaskList scoringSpecimen = new TaskList();

        scoringSpecimen.addTask(new CustomTask(() -> {
            liftPosition = 3;
            outake.autoHighRungPos();
            return true;
        }));

        scoringSpecimen.addTask(new PointTask(new Waypoint(new Pose(0, -850, 0)), ptpController));

        scoringSpecimen.addTask(new DelayTask(500));
        scoringSpecimen.addTask(new CustomTask(() -> {
            outake.openClaw();
            liftPosition = 2.5;
            return true;
        }));

        scoringSpecimen.addTask(new PointTask(new Waypoint(new Pose(0, -400, 0)), ptpController));

        scoringSpecimen.addTask(new CustomTask(() -> {
            outake.transferPos();
            liftPosition = 0;
            return true;
        }));

        queue.addTask(scoringSpecimen);
    }
}
