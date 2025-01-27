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
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides;

public class SpecimenAuto extends AutoSequence {
    private ElapsedTime timer;
    TaskManager manager;
    public boolean samples = false;

    public SpecimenAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, v2CuttleIntake intake, v2CuttleOutake outake,
                        Telemetry telemetry, TaskQueue queue, PTPController ptpController, MotorPositionController liftController,
                        MotorPositionController extController, v2CuttleExtendo extendo, v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager) {
        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt);
        timer = new ElapsedTime();
        this.manager =  manager;
    }

    public int yellowPark(int loopCounter){
        loopCounter += 1;
        if (loopCounter == 25 && samples){
            loopCounter= 0;
            if (intake.getColor() == YELLOW) {
                queue.clear();
                loopCounter = 0;
                specimenPark(0.6);
            }
        }
        return loopCounter;
    }
    public void specimenPark(double speed){
        TaskList park = new TaskList();

        manager.task(park, () -> {
            outake.readyPos();
            intake.off();
            intake.initPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        //tSlop used to be 10, rSlop used to be 0.1
        manager.waypointTask(park, new Pose(-1100, 0, Math.toRadians(-90)),speed,0.2,50,false);

        queue.addTask(park);
    }

    //extendo park
    public void extendoPark(){
        TaskList park = new TaskList();

        manager.task(park, () -> {
            outake.wristDown();
            intake.off();
            intake.initPos();
            extendoPosition = 7.3;
            liftPosition = 0;
        });

        manager.waypointTask(park, new Pose(-700, -200, Math.toRadians(60)),0.9,0.1,10,false);

        queue.addTask(park);
    }

    //TODO:
    public void scoringSpecimen(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset,
                                int x, int y, int r, int x1, int y1, int r1, int x2, int y2, int r2){
        specimenGrab(extOffset,offsetr,offsety,offsetx);
        //teleOp.justTransferSequence();
        scoring(x,y,r,x1,y1,r1,x2,y2,r2);
    }
    public void scoringSpecimenOther(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset,
                                int x, int y, int r, int x1, int y1, int r1, int x2, int y2, int r2){
        specimenGrab(extOffset,offsetr,offsety,offsetx);
        //teleOp.justTransferSequenceMore(x,y,r,x1,y1,r1,x2,y2,r2);
    }

    public void scoringSpecimenFancy(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset){
        specimenGrab(extOffset,offsetr,offsety,offsetx);
        //teleOp.transferSequenceWhileScore(scoreOffset);
    }

    public void scoring(int x, int y, int r, int x1, int y1, int r1, int x2, int y2, int r2) {
        TaskList scoring = new TaskList();
        manager.waypointTask(scoring, new Pose(x, y, Math.toRadians(r)),0.8,0.8,150,true);

        manager.waypointTask(scoring, new Pose(x1, y1, Math.toRadians(r1)),0.8,0.6,150,false);

        manager.delay(scoring, 300);

        manager.task(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        manager.waypointTask(scoring, new Pose(x2, y2, Math.toRadians(r2)),0.8,0.8,150,false);

        manager.task(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        queue.addTask(scoring);
    }

    public void specimenGrab(double extOffset, int offsetr, int offsety, int offsetx) {
        TaskList specimen = new TaskList();

        manager.task(specimen, () -> {
            timer.reset();
            intake.intakeDown();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 2;
            liftPosition = 0;
        });

        manager.waypointTask(specimen, new Pose(-600+offsetx, -350+offsety, Math.toRadians(45+offsetr)),0.9,0.1,10,false);

        //manager.delay(specimen, 200);

        specimen.addTask(new CustomTask(() -> {
            extendoPosition = 4+extOffset;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();
            //intake.setIntakeState(LOOKING);
            if(timer.seconds() > 2) {
                extendoPosition = 0;
                if (timer.seconds() > 3) {
                    timer.reset();
                }
            }

            return true;
            //return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE;
        }));

        manager.task(specimen, () -> {
            extendoPosition = 5;
            //intake.setIntakeState(SECURED);
            intake.clawClose();
        });

        queue.addTask(specimen);
    }

    public void ttSample(int x1, int y1, int r1, int x2, int y2, int r2, double turn) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            extendoPosition = 4;
            intake.intakeDown();
            intake.turntableRight();
            intake.in();
        });

        manager.waypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.4, 0.1, 10, false);

        sample.addTask(new CustomTask(() -> {
            intake.in();
            boolean quit = false;

            //turn = -0.12
            //145
            if (encoderLocalizer.getPos().getR() < Math.toRadians(125)) {
                dt.drive(0.1, 0.2, turn);
            }
            else {
                dt.drive(0, 0, 0);
                quit = true;
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            extendoPosition = 2;
            intake.turntableMiddle();
            intake.clawClose();
        });

        //x2 = -850, y2 = -400, turn = 70
        manager.waypointTask(sample, new Pose(x2, y2, Math.toRadians(r2)), 1, 0.1, 100, false);

        sample.addTask(new CustomTask(() -> {
            intake.clawOpen();
            intake.out();
            extendoPosition = 4;

            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));

        queue.addTask(sample);
    }

    public void ttSample(double extPos, double extPos2, int x1, int y1, int r1, int x2, int y2, int r2, double turn) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            //timer.reset();
            extendoPosition = extPos;
            intake.intakeDown();
            intake.turntableCustom(0.45);
            intake.in();
        });

        manager.waypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.4, 0.1, 10, false);

        manager.task(sample, () -> {
            timer.reset();
            extendoPosition = extPos2;
        });

        manager.delay(sample, 400);

        sample.addTask(new CustomTask(() -> {
            intake.in();
            boolean quit = false;

            //turn = -0.12
            //145
            if (timer.seconds()>3) {
                intake.in();
            }
            else {
                dt.drive(0, 0, 0);
                quit = true;
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            extendoPosition = 2;
            intake.turntableMiddle();
            intake.clawClose();
        });

        //x2 = -850, y2 = -400, turn = 70
        manager.waypointTask(sample, new Pose(x2, y2, Math.toRadians(r2)), 1, 0.1, 100, false);

        sample.addTask(new CustomTask(() -> {
            intake.out();
            intake.clawOpen();
            extendoPosition = 4;
            this.samples = true;

            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));

        queue.addTask(sample);
    }

    public void processSample(int x, int y, double extendo, double extendoOut, double initialAngle, double finalAngle) {
        TaskList sample = new TaskList();
        
        manager.waypointTask(sample, new Pose(x, y, Math.toRadians(initialAngle)));

        manager.task(sample, () -> {
            intake.intakeDown();
            intake.in();
        });

        manager.delay(sample, 200);

        manager.task(sample, () -> {extendoPosition = extendo;});

        manager.delay(sample, 1300);

        manager.task(sample, () -> {extendoPosition = extendoOut;});

        manager.waypointTask(sample, new Pose(x, y, Math.toRadians(finalAngle)));

        manager.task(sample, () -> {intake.out();});

        manager.delay(sample, 200);

        manager.task(sample, () -> {extendoPosition = 0;});

        queue.addTask(sample);
    }
}
