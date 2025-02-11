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

public class SpecimenAuto extends AutoSequence {
    private ElapsedTime timer;
    TaskManager manager;
    public boolean samples = false;

    public SpecimenAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, v2CuttleIntake intake, v2CuttleOutake outake,
                        Telemetry telemetry, TaskQueue queue, PTPController ptpController, MotorPositionController liftController,
                        MotorPositionController extController, v2CuttleExtendo extendo, v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager, v2CuttleHang hang) {
        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt, hang);
        timer = new ElapsedTime();
        this.manager =  manager;
    }

    public int yellowPark(int loopCounter){
        loopCounter += 1;
        if (loopCounter == 5){
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
        manager.waypointTask(park, new Pose(1100, 0, Math.toRadians(90)),speed,0.2,50,false);

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

    public void scoringSpecimenFancy(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset, int y){
        specimenGrab(extOffset,offsetr,offsety,offsetx);
        teleOp.teleOptransferSequence(scoreOffset, y);
    }

    public void intakeOffWall(int x, int y, int r, int x2, int y2, int r2) {
        TaskList scoring = new TaskList();

        manager.task(scoring, ()->{
            extendoPosition = 0;
            intake.armUp();
        });

        manager.waypointTask(scoring, new Pose(x, y, Math.toRadians(r)),0.8,0.8,50,false);
        manager.delay(scoring, 250);

        manager.waypointTask(scoring, new Pose(x2, y2, Math.toRadians(r2)),0.4,0.3,30,false);

        manager.delay(scoring, 150);

        manager.task(scoring, ()->{outake.closeClaw();});

        manager.task(scoring, () -> {
            liftPosition = 2.9;
        });

        queue.addTask(scoring);
    }

    public void score(int x, int y, int r, int x2, int y2, int r2) {
        TaskList posScoring = new TaskList();
        TaskList scoringScoring = new TaskList();

        manager.task(scoringScoring, () -> {
            liftPosition = 3.7;
        });

        manager.waypointTask(posScoring, new Pose(x, y, Math.toRadians(r)), 0.8, 0.8, 150, false);

        manager.waypointTask(posScoring, new Pose(x2, y2, Math.toRadians(r2)), 0.5, 0.8, 15, false);

        manager.task(scoringScoring, () -> {
            outake.specimenFrontReadyPos();
        });

        manager.forkTask(posScoring,scoringScoring);

        TaskList release = new TaskList();

        manager.delay(release, 3);

        manager.task(release, () -> {
            outake.openClaw();
            liftPosition = 0;
        });

        //manager.waypointTask(posScoring, new Pose(x2, y+200, Math.toRadians(r)), 0.8, 0.8, 15, false);

        manager.addTask(release);
    }

    public void scoreOther(int x, int y, int r, double speed) {
        TaskList posScoring = new TaskList();
        TaskList scoringScoring = new TaskList();

        manager.task(scoringScoring, () -> {
            outake.specimenFrontReadyPos();
            liftPosition = 3.8;
        });

        manager.waypointTask(posScoring, new Pose(x, y-200, Math.toRadians(r)), speed, 0.8, 150, false);
        manager.waypointTask(posScoring, new Pose(x, y, Math.toRadians(r)), speed-0.2, 0.6, 150, false);


        manager.forkTask(posScoring,scoringScoring);

        TaskList release = new TaskList();

        manager.delay(release, 250);

        manager.task(release, () -> {
            outake.openClaw();
            liftPosition = 3;
        });

        manager.delay(release, 200);

        manager.waypointTask(posScoring, new Pose(x, y-200, Math.toRadians(r)), 0.8, 0.6, 150, false);

        manager.addTask(release);
    }

    public void specimenGrab(double extOffset, int offsetr, int offsety, int offsetx) {
        TaskList specimen = new TaskList();

        manager.task(specimen, () -> {
            timer.reset();
            intake.intakeDown();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 0;
            liftPosition = 0;
        });
        manager.waypointTask(specimen, new Pose(-450+offsetx, -500+offsety, Math.toRadians(60+offsetr)),0.9,0.1,10,false);

        //manager.delay(specimen, 200);

        specimen.addTask(new CustomTask(() -> {
            extendoPosition = 5+extOffset;
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
            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE;
        }));

        manager.task(specimen, () -> {
            extendoPosition = 5;
            //intake.setIntakeState(SECURED);
            intake.clawClose();
        });

        queue.addTask(specimen);
    }

    public void ttSampleOther(double extIn, int extOut, int x1, int y1, int r1, int x2, int y2, int r2, double turn) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            extendoPosition = extIn;
            intake.intakeDown();
            intake.turntableRight();
            intake.in();
        });

        manager.waypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.35, 0.1, 10, false);

        sample.addTask(new CustomTask(() -> {
            intake.in();
            boolean quit = false;

            //turn = -0.12
            //145
            if (encoderLocalizer.getPos().getR() < Math.toRadians(145)) {
                dt.drive(0.15, 0.3, turn);
            }
            else {
                dt.drive(0, 0, 0);
                quit = true;
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            extendoPosition = 1;
            intake.turntableMiddle();
            intake.clawClose();
        });

        //x2 = -850, y2 = -400, turn = 70
        manager.waypointTask(sample, new Pose(x2, y2, Math.toRadians(r2)), 0.7, 0.1, 100, false);

        sample.addTask(new CustomTask(() -> {
            intake.armMiddle();
            intake.clawOpen();
            intake.out();
            extendoPosition = extOut;

            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));

        manager.task(sample, () -> {
            extendoPosition = 2;
        });

        manager.delay(sample,300);

        queue.addTask(sample);
    }

    public void sampleSweep(double extPos, int x1, int y1, int r1, int x2, int y2, int r2, int delay_time) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            extendoPosition = extPos;
            intake.intakeDown();
            intake.turntableMiddle();
        });

        manager.waypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.35, 0.5, 10, false);

        //manager.waypointTask(sample, new Pose(x2, y2, Math.toRadians(r2)), 0.2, 0.1, 100, false);

        manager.task(sample, () -> {
            dt.drive(0,0.33,0.3);
        });

        manager.delay(sample, delay_time);
        manager.task(sample, () -> {
           extendoPosition = 3;
        });

        queue.addTask(sample);
    }

    public void sweepSetup(int x1, int y1, int r1) {
        TaskList sweepSetup = new TaskList();

        manager.waypointTask(sweepSetup, new Pose(x1, y1, Math.toRadians(r1)), 1, 0.5, 50, false);

        manager.task(sweepSetup, () -> {
            liftPosition = 0;
            //extendoPosition = 0;
        });

        queue.addTask(sweepSetup);
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

        manager.delay(sample,200);

        manager.task(sample, () -> {
            outake.scorePosRight();
        });

        queue.addTask(sample);
    }

    public void scoreSetup(){
        TaskList setup = new TaskList();

        manager.task(setup, ()->{
            extendoPosition = 0;
            intake.armUp();
            liftPosition = 2.8;
            outake.backIntakePos();
            outake.openClaw();
        });

        manager.addTask(setup);
    }

    public void ttSample(double extPos, double extPos2, int x1, int y1, int r1, int x2, int y2, int r2, double extOut) {
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

        manager.delay(sample, 200);

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
            intake.armMiddle();
            intake.out();
            intake.clawOpen();
            extendoPosition = extOut;
            outake.backIntakePos();
            outake.openClaw();

            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));

        manager.task(sample, () -> {
            extendoPosition = 2;
        });

        manager.delay(sample, 200);

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
