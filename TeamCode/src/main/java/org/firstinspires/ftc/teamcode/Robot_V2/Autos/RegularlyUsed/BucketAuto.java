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
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSweep;

public class BucketAuto extends AutoSequence {
    private ElapsedTime timer;
    TaskManager manager;


    public BucketAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, v2CuttleIntake intake, v2CuttleOutake outake,
                      Telemetry telemetry, TaskQueue queue, PTPController ptpController, MotorPositionController liftController,
                      MotorPositionController extController, v2CuttleExtendo extendo, v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager, v2CuttleHang hang, v2CuttleSweep sweeper) {
        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt, hang, sweeper);
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

        manager.task(park, () -> {
            dt.drive(-0.5,0,0);
            outake.parkPos();
        });

        queue.addTask(park);
    }

    public void scoringBuckets(int inX, int inY, double inR, double extPos, int scoreX, int scoreY, double scoreR, double r, int endX, int endY){
        intakeSample(inX,inY,inR,extPos);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR);
        scoreSample(scoreX, scoreY, scoreR, r, endX, endY);
    }

    public void middle(int x, int y, double r,int x2, int y2, double r2, int scoreX, int scoreY, double scoreR, double r3, int endX, int endY) {
        messUpMiddle(x, y, r, x2, y2, r2);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR);
        scoreSample(scoreX, scoreY, scoreR, r3, endX, endY);
    }

    public void messUpMiddle(int x, int y, double r,int x2, int y2, double r2){
        TaskList mess = new TaskList();

        manager.waypointTask(mess, new Pose(x, y, Math.toRadians(r)),0.9,0.5,100,false);

        manager.waypointTask(mess, new Pose(x2, y2, Math.toRadians(r2)),0.9,0.5,100,false);

        manager.task(mess, () -> {
            extendoPosition = 0;
            intake.armUp();
            outake.readyPos();
            sweeper.broomOut();
        });

        manager.delay(mess, 400);

        manager.task(mess, () -> {
            sweeper.broomIn();
        });

       manager.delay(mess, 400);

       manager.waypointTask(mess, new Pose(x2, y2-10, Math.toRadians(r2)),0.9,0.5,100,false);

        mess.addTask(new CustomTask(() -> {
            boolean quit = false;
            extendoPosition = 0;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            //if (timer.seconds() > 5) {quit = true;}
            if(intake.getColor() == BLUE){
                intake.out();
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || quit;
        }));

        manager.task(mess, () -> {
            intake.clawClose();
        });

        queue.addTask(mess);
    }

    public void scoreSample(double x, double y, double r1, double r2, double endX, double endY) {
        TaskList scoringSample = new TaskList();

        manager.waypointTask(scoringSample, new Pose(x+20, y, Math.toRadians(r1)),0.8,0.1,10,false);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        manager.delay(scoringSample, 500);

        manager.task(scoringSample, () -> {
            outake.openClaw();
        });

        manager.waypointTask(scoringSample, new Pose(x-20, y, Math.toRadians(r1)),0.6,0.1,20,false);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });

        manager.waypointTask(scoringSample, new Pose(endX,endY, Math.toRadians(r2)),0.8,0.1,150,false);

        queue.addTask(scoringSample);
    }

    public void intakeSample(double x, double y, double deg, double extPos) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            timer.reset();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 0;
            liftPosition = 0;
            //intake.turntableCustom(0.7);
        });

        manager.waypointTask(sample, new Pose(x, y, Math.toRadians(deg)),0.9,0.1,10,false);

        //changed from 400
        //manager.delay(sample, 200);

        sample.addTask(new CustomTask(() -> {
            boolean quit = false;
            extendoPosition = extPos;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            if (timer.seconds() > 2.5) {quit = true;}

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            extendoPosition = 0;
            intake.clawClose();
        });

        queue.addTask(sample);
    }

    public void scoreFirstSample(int xPos, int yPos, double r, int finishxpos, int finishy, double finishr) {
        TaskList scoringSample = new TaskList();

        manager.task(scoringSample, () -> {
            liftPosition = 14;
        });

        //x used to be -980, -1000 still works
        manager.waypointTask(scoringSample, new Pose(xPos, yPos, Math.toRadians(r)),0.6,0.6,20,false);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        manager.delay(scoringSample, 500);

        manager.task(scoringSample, () -> {
            outake.openClaw();});

        manager.delay(scoringSample, 200);

       manager.task(scoringSample, () -> {
           outake.readyPos();
            liftPosition = 0;
        });

        manager.waypointTask(scoringSample, new Pose(finishxpos, finishy, Math.toRadians(finishr)),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }

}
