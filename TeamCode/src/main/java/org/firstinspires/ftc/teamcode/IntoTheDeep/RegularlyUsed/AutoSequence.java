package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;

public class AutoSequence extends CuttleInitOpMode {
    public boolean test;
    public String color;
    public String side;
    ElapsedTime autoTimer;

    public AutoSequence(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, CuttleIntake intake, CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                 PTPController ptpController, MotorPositionController liftController, MotorPositionController extController,
                 CuttleExtendo extendo, CuttleSlides lift, CuttleDT dt) {
        //Initializing values
        this.otosLocalizer = otos;
        this.encoderLocalizer = encoderLocalizer;
        this.intake = intake;
        this.outake = outake;
        this.telemetry = telemetry;
        this.queue = queue;
        this.ptpController = ptpController;
        this.liftPosController = liftController;
        this.extendoPosController = extController;
        this.lift = lift;
        this.extendo = extendo;
        this.dt = dt;

        this.test = Setup.test;
        this.color = Setup.color;
        this.side = Setup.side;
    }

    public void specimenPark(){
        TaskList park = new TaskList();

        task.addWaypointTask(park, new Pose(-800, 200, Math.toRadians(90)),0.8,0.5,150,false);

        task.addIntakeTask(park, () -> {
            outake.readyPos();
            intake.off();
            intake.initPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        queue.addTask(park);
    }

    public void scoringSpecimen(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset){
        specimen(extOffset,offsetr,offsety,offsetx);
        teleOp.transferSequence();
        scoring(scoreOffset);
    }

    public void scoring3(){
        scoringSpecimen(0, -4, 0,0,0);
        scoringSpecimen(0.25, -6,0,0,20);
        scoringSpecimen(0.75,-7,0, -10,50);
    }

    public void specimen(double extOffset, int offsetr, int offsety, int offsetx) {
        TaskList specimen = new TaskList();

        autoTimer.reset();
        specimen.addTask(new CustomTask(() -> {
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 2;
            liftPosition = 0;
            return true;
        }));

        task.addWaypointTask(specimen, new Pose(-600+offsetx, -350+offsety, Math.toRadians(45+offsetr)),0.9,0.1,10,false);

        specimen.addTask(new CustomTask(() -> {
            extendoPosition = 4+extOffset;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE;
        }));

        task.addIntakeTask(specimen, () -> {
            extendoPosition = 5;
            intake.clawClose();
        });

        queue.addTask(specimen);
    }

    public void scoring(int extraX) {
        TaskList scoring = new TaskList();

        task.addWaypointTask(scoring, new Pose(-100+extraX, -650, Math.toRadians(0)),0.9,0.5,150,true);
        task.addWaypointTask(scoring, new Pose(-100+extraX, -600, Math.toRadians(0)),0.9,0.5,150,false);

        task.addDelayTask(scoring, 300);

       task.addIntakeTask(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        task.addDelayTask(scoring, 300);

        task.addWaypointTask(scoring, new Pose(-300, -400, 0),0.8,0.8,150,false);

        task.addIntakeTask(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        queue.addTask(scoring);
    }

    public void ttSample(){
        fistSampleTT();
        secondSampleTT();
        thridSampleTT();
    }

    public void fistSampleTT() {
        ttdriving(-750, -590,90);
    }
    public void secondSampleTT(){ttdriving(-1000, -580,90);}
    public void thridSampleTT(){
        ttdriving(-1260, -610,100);
    }

    public void ttdriving(int x1, int y1, int r1) {
        //gets given positions to drive the robot
        TaskList sample = new TaskList();

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 4;
            intake.turntableRight();
            intake.in();
            return true;
        }));

        task.addWaypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.8, 0.1, 10, false);

        sample.addTask(new CustomTask(() -> {
            intake.in();
            boolean quit = false;
            if (encoderLocalizer.getPos().getR() < Math.toRadians(145)) {
                dt.drive(0, 0.1, -0.25);
            } else {
                dt.drive(0, 0, 0);
                quit = true;
            }
            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 4;
            intake.turntableLeft();
            return true;
        }));

        task.addWaypointTask(sample, new Pose(-900, -400, Math.toRadians(70)), 1, 0.5, 100, false);

        sample.addTask(new CustomTask(() -> {
            intake.out();
            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));

        queue.addTask(sample);
    }

    //score first specimen during auto
    public void firstSpecimen() {
        TaskList scoringSpecimen = new TaskList();

        task.addIntakeTask(scoringSpecimen, () -> {
            liftPosition = 2.9;
            outake.autoAutoHighRungPos();
            extendoPosition = 3;
        });

        task.addWaypointTask(scoringSpecimen, new Pose(-100, -750, 0),0.9,0.5,150,false);
        task.addDelayTask(scoringSpecimen, 200);
        task.addIntakeTask(scoringSpecimen, () -> {
            outake.openClaw();
            liftPosition = 2;
            extendoPosition = 3;
        });
        task.addDelayTask(scoringSpecimen, 300);

        task.addWaypointTask(scoringSpecimen, new Pose(-150, -400, Math.toRadians(50)),0.9,0.5,150,false);

        task.addIntakeTask(scoringSpecimen, () -> {
            outake.transferPos();
            liftPosition = 0;
            extendoPosition = 0;
            outake.readyPos();
        });

        queue.addTask(scoringSpecimen);
    }
}
