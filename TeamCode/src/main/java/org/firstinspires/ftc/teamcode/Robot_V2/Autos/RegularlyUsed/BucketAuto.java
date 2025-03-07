package org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.YELLOW;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
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
    private ElapsedTime failsafeTimer;
    private ElapsedTime newTimer;
    TaskManager manager;
    int count = 0;

    boolean park = true;


    public BucketAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, v2CuttleIntake intake, v2CuttleOutake outake,
                      Telemetry telemetry, TaskQueue queue, PTPController ptpController, MotorPositionController liftController,
                      MotorPositionController extController, v2CuttleExtendo extendo, v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager, v2CuttleHang hang, v2CuttleSweep sweeper) {
        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt, hang, sweeper);
        timer = new ElapsedTime();
        failsafeTimer = new ElapsedTime();
        newTimer = new ElapsedTime();

        this.manager = manager;
    }
    public void park(int x2, int y2, double r2, int x, int y, double r){
        TaskList park = new TaskList();

        manager.waypointTask(park, new Pose(x2, y2, Math.toRadians(r2)),1,0.5,100,false);

        manager.task(park, () -> {
            intake.turntableMiddle();
            //outake.readyPos();
            //hang.parkHeight();
            intake.armUp();
            extendoPosition = 0;
            liftPosition = 0;
            outake.parkPos();
        });

        manager.waypointTask(park, new Pose(x, y, Math.toRadians(r)),1,0.5,100,false);

        manager.task(park, () -> {
            dt.drive(-0.5,0,0);
        });

        queue.addTask(park);
    }

    public void park(int x, int y, double r){
        TaskList park = new TaskList();

        manager.task(park, () -> {
            intake.turntableMiddle();
            //outake.readyPos();
            //hang.parkHeight();
            intake.armUp();
            extendoPosition = 0;
            liftPosition = 0;
            outake.parkPos();
        });

        manager.waypointTask(park, new Pose(x, y, Math.toRadians(r)),0.9,0.5,100,false);

        manager.task(park, () -> {
            dt.drive(-0.2,0,0);
        });

        queue.addTask(park);
    }

    public void scoringBuckets(int inX, int inY, double inR, double extPos, int scoreX, int scoreY, double scoreR, double r, int endX, int endY){
        intakeSample(inX,inY,inR,extPos);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR);
        scoreSample(scoreX, scoreY, scoreR, r, endX, endY);
    }

    public void scoringBuckets2(int inX, int inY, double inR, double extPos, int scoreX, int scoreY, double scoreR, int time, double extLast, double time2){
         intakeSample2(inX,inY,inR,extPos, time2);
         teleOp.bucketTransfer(scoreX, scoreY, scoreR);
         scoreSample2(scoreX, scoreY, scoreR, time, extLast);
    }

    public void scoringBucketsLast(int inX, int inY, double inR, double extPos, int scoreX, int scoreY, double scoreR, int time, double time2){
        intakeSampleLast(inX,inY,inR,extPos, time2);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR);
        scoreSampleForLast(scoreX, scoreY, scoreR, time);
    }

    public void middle(int x, int y, double r,int x2, int y2, double r2, int scoreX, int scoreY, double scoreR, double r3, int endX, int endY, v2CuttleIntake.Color in, v2CuttleIntake.Color out) {
        messUpMiddleRed(x, y, r, x2, y2, r2, in, out);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR);
        scoreSample(scoreX, scoreY, scoreR, r3, endX, endY);
    }

    public void middle3(int x, int y, double r,int x2, int y2, double r2, int scoreX, int scoreY, double scoreR, v2CuttleIntake.Color in, v2CuttleIntake.Color out) {
        messUpMiddleRed3(x, y, r, x2, y2, r2, in, out);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR, 0.6);
        scoreSampleForLastOther(scoreX, scoreY, scoreR,200);
    }

    public void middle2(int x, int y, double r,int x2, int y2, double r2, int scoreX, int scoreY, double scoreR, v2CuttleIntake.Color in, v2CuttleIntake.Color out) {
        messUpMiddleRed(x, y, r, x2, y2, r2, in, out);
        teleOp.bucketTransfer(scoreX, scoreY, scoreR, 0.6);
        scoreSampleForLastOther(scoreX, scoreY, scoreR,200);
    }

    public void messUpMiddleRed(int x, int y, double r, int x2, int y2, double r2, v2CuttleIntake.Color colorIN, v2CuttleIntake.Color colorOUT){
        TaskList mess = new TaskList();

        manager.waypointTask(mess, new Pose(x, y, Math.toRadians(r)),0.9,0.2,75,false);

        //manager.waypointTask(mess, new Pose(x2, y2, Math.toRadians(r2)),0.9,0.5,100,false);
        mess.addTask(new CustomTask(() -> {
            dt.drive(0.7,0,0);
            return true;
        }));

        manager.delay(mess, 600);

        mess.addTask(new CustomTask(() -> {
            dt.drive(0,0,0);
            return true;
        }));

        /*mess.addTask(new CustomTask(() -> {
            dt.drive(0.3,0,0);
            return true;
        }));

        manager.delay(mess, 200);
        mess.addTask(new CustomTask(() -> {
            dt.drive(0,0,0);
            return true;
        }));*/

        manager.task(mess, () -> {
            extendoPosition = 0;
            intake.armUp();
            outake.readyPos();
            sweeper.broomOut();
        });

        manager.delay(mess, 400);

        manager.task(mess, () -> {
            intake.in();
            sweeper.broomIn();
        });

        manager.delay(mess, 300);

        manager.task(mess, () -> {
            extendoPosition = 0;
            intake.armUp();
            outake.readyPos();
            sweeper.broomOut();
        });

        manager.delay(mess, 300);

        manager.task(mess, () -> {
            timer.reset();
        });

        manager.task(mess, () -> {
            intake.clawOpen();
            intake.intakeDown();
        });

        manager.delay(mess, 200);

        manager.task(mess, () -> {
            timer.reset();
            intake.in();
            extendoPosition = 3;
            count = 0;
            failsafeTimer.reset();
        });

        mess.addTask(new CustomTask(() -> {
            //turn = -0.12
            //145

            if (intake.getColor() == colorOUT) {
                newTimer.reset();
                intake.out();

                while (newTimer.seconds() <= 0.5){
                    intake.out();
                }

                extendoPosition = 0;
            }

            else if(timer.seconds() >= 0  && timer.seconds() < 1.5  && intake.getColor() == null){

                if(timer.seconds() >= 0.5){
                    intake.in();
                    intake.turntableMiddle();
                    extendoPosition = 3;
                }

                else{
                    intake.turntableMiddle();
                    extendoPosition = 0;
                    intake.out();
                }
            }

            else if(timer.seconds() >= 1.5  && timer.seconds() < 3  && intake.getColor() == null){

                if(timer.seconds() >= 2){
                    intake.in();
                    intake.turntableLeft();
                    //intake.turntableCustom(0.35);
                    extendoPosition = 2;
                }

                else{
                    intake.turntableLeft();
                    extendoPosition = 0;
                    intake.out();
                }
            }

            else if(timer.seconds() >= 3  && timer.seconds() < 4.5 && intake.getColor() == null){
                if(timer.seconds() >= 3.5){
                    intake.in();
                    intake.turntableRight();
                    extendoPosition = 2;
                }

                else{
                    intake.turntableRight();
                    extendoPosition = 0;
                    intake.out();
                }
            }

            if(failsafeTimer.seconds() >= 4.5){
                if(timer.seconds() <= 5){
                    intake.out();
                    intake.turntableMiddle();
                    extendoPosition = 0;
                }

                else if(timer.seconds() >= 5 && timer.seconds() < 5.5){
                    intake.off();
                    intake.armUp();
                }

                else {
                    queue.clear();

                    queue.addTask(new CustomTask(() -> {
                        sweeper.broomSet(0.4);

                        return true;
                    }));

                    queue.addTask(new PointTask(new Waypoint(new Pose(-1500, 0, Math.toRadians(180)), 1,0.5,100, false), ptpController));

                    queue.addTask(new CustomTask(() -> {
                        sweeper.broomIn();
                        intake.turntableMiddle();
                        //outake.readyPos();
                        //hang.parkHeight();
                        intake.armUp();
                        extendoPosition = 0;
                        liftPosition = 0;
                        outake.parkPos();

                        return true;
                    }));

                    queue.addTask(new PointTask(new Waypoint(new Pose(-1400, 350, Math.toRadians(180)), 1,0.5,100, false), ptpController));

                    queue.addTask(new CustomTask(() -> {
                        dt.drive(-0.5,0,0);
                        return true;
                    }));

                    queue.addTask(new DelayTask(60000));
                    return true;
                }
            }

            return intake.getColor() == YELLOW || intake.getColor() == colorIN;
        }));

        manager.task(mess, () -> {
            intake.in();
        });

        manager.delay(mess, 200);

        manager.task(mess, () -> {
            intake.clawClose();
        });

        manager.delay(mess, 200);

        manager.task(mess, () -> {
            intake.off();
            intake.armUp();
            sweeper.broomSet(0.4);
        });
        queue.addTask(mess);
    }

    public void messUpMiddleRed3(int x, int y, double r, int x2, int y2, double r2, v2CuttleIntake.Color colorIN, v2CuttleIntake.Color colorOUT){
        TaskList mess = new TaskList();

        manager.waypointTask(mess, new Pose(x, y, Math.toRadians(r)),0.9,0.2,110,false);

        //manager.waypointTask(mess, new Pose(x2, y2, Math.toRadians(r2)),0.9,0.5,100,false);
        mess.addTask(new CustomTask(() -> {
            extendoPosition = 0;
            intake.armUp();
            outake.readyPos();
            intake.in();
            dt.drive(0.7,0,0);
            return true;
        }));

        manager.delay(mess, 600);

        mess.addTask(new CustomTask(() -> {
            dt.drive(0,0,0);
            return true;
        }));

        manager.task(mess, () -> {
            intake.clawOpen();
            intake.intakeDown();
        });

        manager.task(mess, () -> {
            intake.turntableMiddle();
        });

        manager.delay(mess, 200);

        manager.task(mess, () -> {
            timer.reset();
            intake.in();
            failsafeTimer.reset();
        });

        mess.addTask(new CustomTask(() -> {

            if (intake.getColor() == colorOUT) {
                newTimer.reset();
                intake.out();

                while (newTimer.seconds() <= 0.5){
                    intake.out();
                }

                extendoPosition = 0;
            }

            else if(timer.seconds() >= 0  && timer.seconds() < 1.5  && intake.getColor() == null){

                if(timer.seconds() >= 0.5){
                    intake.in();
                    intake.turntableMiddle();
                    extendoPosition = 4.5;
                }

                else{
                    intake.turntableMiddle();
                    extendoPosition = 0;
                    intake.out();
                }
            }

            else if(timer.seconds() >= 1.5  && timer.seconds() < 3  && intake.getColor() == null){

                if(timer.seconds() >= 2){
                    intake.in();
                    intake.turntableRight();
                    //intake.turntableCustom(0.35);
                    extendoPosition = 3;
                }

                else{
                    intake.turntableMiddle();
                    extendoPosition = 0;
                    intake.out();
                }
            }

            else if(timer.seconds() >= 3  && timer.seconds() < 4.5 && intake.getColor() == null){

                if(timer.seconds() >= 3.5){
                    intake.in();
                    intake.turntableLeft();
                    extendoPosition = 3;
                }

                else{
                    intake.turntableLeft();
                    extendoPosition = 0;
                    intake.out();
                }
            }

            if(failsafeTimer.seconds() >= 4.5){
                if(timer.seconds() <= 5){
                    intake.out();
                    intake.turntableMiddle();
                    extendoPosition = 0;
                }

                else if(timer.seconds() >= 5 && timer.seconds() < 5.5){
                    intake.off();
                    intake.armUp();
                }

                else {
                    queue.clear();

                    queue.addTask(new CustomTask(() -> {
                        sweeper.broomSet(0.4);

                        return true;
                    }));

                    queue.addTask(new PointTask(new Waypoint(new Pose(-1500, 0, Math.toRadians(180)), 1,0.5,100, false), ptpController));

                    queue.addTask(new CustomTask(() -> {
                        sweeper.broomIn();
                        intake.turntableMiddle();
                        //outake.readyPos();
                        //hang.parkHeight();
                        intake.armUp();
                        extendoPosition = 0;
                        liftPosition = 0;
                        outake.parkPos();

                        return true;
                    }));

                    queue.addTask(new PointTask(new Waypoint(new Pose(-1400, 350, Math.toRadians(180)), 1,0.5,100, false), ptpController));

                    queue.addTask(new CustomTask(() -> {
                        dt.drive(-0.5,0,0);
                        return true;
                    }));

                    queue.addTask(new DelayTask(60000));
                    return true;
                }
            }

            return intake.getColor() == YELLOW || intake.getColor() == colorIN;
        }));

        manager.task(mess, () -> {
            intake.in();
        });

        manager.delay(mess, 200);

        manager.task(mess, () -> {
            intake.clawClose();
        });

        manager.delay(mess, 200);

        manager.task(mess, () -> {
            intake.off();
            intake.armUp();
        });

        queue.addTask(mess);
    }

    public void scoreSample(double x, double y, double r1, double r2, double endX, double endY) {
        TaskList scoringSample = new TaskList();

        manager.waypointTask(scoringSample, new Pose(x, y, Math.toRadians(r1)),0.8,0.1,10,false);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        manager.waypointTask(scoringSample, new Pose(x+70, y, Math.toRadians(r1)),0.6,0.1,10,false);


        //manager.delay(scoringSample, 100);

        manager.task(scoringSample, () -> {
            outake.openClaw();
        });

       manager.waypointTask(scoringSample, new Pose(x, y, Math.toRadians(r1)),0.6,0.1,20,false);

        //manager.waypointTask(scoringSample, new Pose(endX,endY, Math.toRadians(r2)),0.8,0.6,150,false);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });

        manager.waypointTask(scoringSample, new Pose(endX,endY, Math.toRadians(r2)),0.8,0.6,150,false);

        queue.addTask(scoringSample);
    }

    public void scoreSampleForLast(double x, double y, double r1, int time) {
        TaskList scoringSample = new TaskList();

        manager.waypointTask(scoringSample, new Pose(x, y, Math.toRadians(r1)),0.8,0.2,20,false);

        manager.task(scoringSample, () -> {
            sweeper.broomIn();
            outake.scorePosMid();
        });

        manager.task(scoringSample, () -> {
            dt.drive(-0.2,0, 0);
        });

        manager.delay(scoringSample, time);

        manager.task(scoringSample, () -> {
            dt.drive(0,0,0);
        });

        //manager.delay(scoringSample, 50);

        manager.task(scoringSample, () -> {
            outake.openClaw();
            intake.clawOpen();
        });

        manager.waypointTask(scoringSample, new Pose(x, y, Math.toRadians(r1)),0.8,0.1,20,false);

        manager.task(scoringSample, () -> {
            outake.readyPos();
        });

        manager.delay(scoringSample, 100);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
        });

        queue.addTask(scoringSample);
    }

    public void scoreSampleForLastOther(double x, double y, double r1, int time) {
        TaskList scoringSample = new TaskList();

        manager.task(scoringSample, () -> {
            sweeper.broomIn();
            outake.scorePosMid();
        });

        manager.task(scoringSample, () -> {
            dt.drive(-0.2,0, 0);
        });

        manager.delay(scoringSample, time);

        manager.task(scoringSample, () -> {
            dt.drive(0,0,0);
        });

        //manager.delay(scoringSample, 50);

        manager.task(scoringSample, () -> {
            outake.openClaw();
            intake.clawOpen();
        });

        manager.waypointTask(scoringSample, new Pose(x, y, Math.toRadians(r1)),0.8,0.1,20,false);

        manager.task(scoringSample, () -> {
            outake.readyPos();
        });

        manager.delay(scoringSample, 100);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
        });

        queue.addTask(scoringSample);
    }

    public void scoreSample2(double x, double y, double r1, int time, double extLast) {
        TaskList scoringSample = new TaskList();

        manager.waypointTask(scoringSample, new Pose(x, y, Math.toRadians(r1)),0.8,0.1,10,false);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        manager.task(scoringSample, () -> {
            dt.drive(-0.2,0, 0);
        });

        manager.delay(scoringSample, time);

        manager.task(scoringSample, () -> {
            dt.drive(0,0,0);
        });

        manager.task(scoringSample, () -> {
            extendoPosition = extLast;
            intake.intakeDown();
            intake.clawOpen();
            intake.in();
        });

        //manager.delay(scoringSample, 200);

        manager.task(scoringSample, () -> {
            outake.openClaw();
        });

        manager.waypointTask(scoringSample, new Pose(x, y, Math.toRadians(r1)),0.8,0.1,20,false);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
            intake.intakeDown();
            intake.clawOpen();
            intake.in();
        });

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

    public void intakeSample2(double x, double y, double deg, double extPos, double time) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            timer.reset();
            intake.turntableMiddle();
            intake.in();
            //extendoPosition = 0;
            liftPosition = 0;
            intake.intakeDown();
            intake.clawOpen();
        });

        manager.waypointTask(sample, new Pose(x, y, Math.toRadians(deg)),0.9,0.1,10,false);

        sample.addTask(new CustomTask(() -> {
            boolean quit = false;
            extendoPosition = extPos;
            //intake.in();
            //intake.intakeDown();

            if (timer.seconds() > time) {
                quit = true;
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            intake.in();
        });

        manager.delay(sample, 200);

        manager.task(sample, () -> {
            extendoPosition = 0;
            intake.clawClose();
        });

        queue.addTask(sample);
    }

    public void intakeSampleLast(double x, double y, double deg, double extPos, double time) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            timer.reset();
            intake.turntableMiddle();
            intake.in();
            //extendoPosition = 0;
            liftPosition = 0;
            intake.intakeDown();
            intake.clawOpen();
        });

        manager.waypointTask(sample, new Pose(x, y, Math.toRadians(deg)),0.9,0.2,10,false);

        sample.addTask(new CustomTask(() -> {
            boolean quit = false;
            extendoPosition = extPos;
            //intake.in();
            //intake.intakeDown();

            if (timer.seconds() > time) {
                quit = true;
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            intake.in();
        });

        manager.delay(sample, 200);

        manager.task(sample, () -> {
            extendoPosition = 0;
            intake.clawClose();
        });

        queue.addTask(sample);
    }

    public void scoreFirstSample2(int xPos, int yPos, double r, int time) {
        TaskList scoringSample = new TaskList();

        manager.task(scoringSample, () -> {
            liftPosition = 14;
        });

        //x used to be -980, -1000 still works
        manager.waypointTask(scoringSample, new Pose(xPos, yPos, Math.toRadians(r)),0.8,0.6,20,false);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        manager.task(scoringSample, () -> {
            dt.drive(-0.2,-0.2, 0);
        });

        manager.delay(scoringSample, time);

        manager.task(scoringSample, () -> {
            dt.drive(0,0,0);
        });

        manager.delay(scoringSample, 200);

        manager.task(scoringSample, () -> {
            outake.openClaw();
            intake.clawOpen();
            intake.intakeDown();
            intake.in();
            extendoPosition = 1;
        });
        manager.delay(scoringSample, 100);

        manager.waypointTask(scoringSample, new Pose(xPos, yPos, Math.toRadians(r)),0.8,0.6,20,false);

       manager.task(scoringSample, () -> {
           outake.readyPos();
           liftPosition = 0;
        });

        queue.addTask(scoringSample);
    }

    public void scoreFirstSample3(int xPos, int yPos, double r, int time, double ext) {
        TaskList scoringSample = new TaskList();

        manager.task(scoringSample, () -> {
            liftPosition = 14;
        });

        //x used to be -980, -1000 still works
        manager.waypointTask(scoringSample, new Pose(xPos, yPos, Math.toRadians(r)),0.8,0.6,20,false);

        manager.task(scoringSample, () -> {
            outake.scorePosMid();
        });

        manager.task(scoringSample, () -> {
            dt.drive(-0.2,-0.2, 0);
        });

        manager.delay(scoringSample, time);

        manager.task(scoringSample, () -> {
            dt.drive(0,0,0);
        });

        manager.delay(scoringSample, 200);

        manager.task(scoringSample, () -> {
            outake.openClaw();
            intake.clawOpen();
            intake.intakeDown();
            intake.in();
            extendoPosition = ext;
        });
        manager.delay(scoringSample, 100);

        manager.waypointTask(scoringSample, new Pose(xPos, yPos, Math.toRadians(r)),0.8,0.6,20,false);

        manager.task(scoringSample, () -> {
            outake.readyPos();
            liftPosition = 0;
        });

        queue.addTask(scoringSample);
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

        manager.delay(scoringSample, 400);

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
