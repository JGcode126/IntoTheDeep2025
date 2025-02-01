package org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.YELLOW;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleDT;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleHang;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides;

public class BucketAuto extends AutoSequence {
    private ElapsedTime timer;
    TaskManager manager;


    public BucketAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, v2CuttleIntake intake, v2CuttleOutake outake,
                      Telemetry telemetry, TaskQueue queue, PTPController ptpController, MotorPositionController liftController,
                      MotorPositionController extController, v2CuttleExtendo extendo, v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager, v2CuttleHang hang) {
        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt, hang);
        timer = new ElapsedTime();
        this.manager = manager;
    }
    public void park(int x, int y, double r){
        TaskList park = new TaskList();
        manager.task(park, () -> {
            intake.turntableMiddle();
            outake.readyPos();
            hang.parkHeight();
            intake.armUp();
            extendoPosition = 0;
            liftPosition = 0;
        });

        manager.waypointTask(park, new Pose(x, y, Math.toRadians(r)),0.9,0.5,100,false);

        manager.task(park, () -> {dt.drive(-0.2,0,0);});

        queue.addTask(park);
    }

    public void scoringBuckets(int inX, int inY, double inR, int scoreX, int scoreY, double scoreR, double r){
        intakeSample(inX,inY,inR);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR);
        scoreSample(scoreX, scoreY, scoreR, r);
    }

    public void scoreSample(double x, double y, double r1, double r2) {
        TaskList scoringSample = new TaskList();

        manager.waypointTask(scoringSample, new Pose(x+20, y, Math.toRadians(r1)),0.6,0.1,10,false);

        //changed from 300
        manager.delay(scoringSample, 200);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        //changed from 400
        manager.delay(scoringSample, 400);

        manager.task(scoringSample, () -> {
            outake.openClaw();
        });

        manager.delay(scoringSample, 200);

        manager.waypointTask(scoringSample, new Pose(x-20, y, Math.toRadians(r1)),0.6,0.1,10,false);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });

        manager.waypointTask(scoringSample, new Pose(-300, -780, Math.toRadians(r2)),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }


    public void intakeSample(double x, double y, double deg) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            timer.reset();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 0;
            liftPosition = 0;
        });

        manager.waypointTask(sample, new Pose(x, y, Math.toRadians(deg)),0.9,0.1,10,false);

        manager.delay(sample, 400);

        sample.addTask(new CustomTask(() -> {
            boolean quit = false;
            extendoPosition = 5;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            if (timer.seconds() > 3.5) {quit = true;}

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            extendoPosition = 0;
            intake.clawClose();
        });

        queue.addTask(sample);
    }

    public void scoreFirstSample(int xPos, int yPos, double r, int finishxpos, int finishy) {
        TaskList scoringSample = new TaskList();

        manager.task(scoringSample, () -> {
            liftPosition = 14;
        });

        //x used to be -980, -1000 still works
        manager.waypointTask(scoringSample, new Pose(xPos, yPos, Math.toRadians(r)),0.6,0.6,10,false);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        manager.delay(scoringSample, 300);

        manager.task(scoringSample, () -> {
            outake.openClaw();});

        manager.delay(scoringSample, 200);

       manager.task(scoringSample, () -> {
           outake.readyPos();
            liftPosition = 0;
        });

        manager.waypointTask(scoringSample, new Pose(finishxpos, finishy, Math.toRadians(r)),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }

}
