package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.Setup.side;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.ForkTask;
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

public class TeleOp extends CuttleInitOpMode{
    public double highChamberPos = 5;
    public double highBucketPos = 14;

    CuttleIntake intake;
    CuttleSlides lift;
    CuttleOutake outake;
    CuttleExtendo extendo;
    CuttleDT dt;
    TaskManager manager;

    public TeleOp(CuttleIntake intake, CuttleOutake outake, CuttleExtendo extendo, CuttleSlides lift, CuttleDT dt, TaskManager manager) {

        this.intake = intake;
        this.lift = lift;
        this.outake = outake;
        this.extendo = extendo;
        this.dt = dt;
        this.manager = manager;

    }

    public void testingIfWorks(){
        TaskList test = new TaskList();
        manager.task(test, ()->{
            extendoPosition = 4;
            liftPosition = 7;
        });
        manager.addTask(test);
    }

    //idk if will work
    public void transferSequence(){
        TaskList transfer = new TaskList();
        TaskList movement = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        manager.waypointTask(movement, new Pose(0, -300, Math.toRadians(0)),0.8,0.5,150,false);

        manager.task(transfer, ()->{
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        manager.delay(transfer, 600);

        manager.task(transfer, ()->{outake.transferPos();});

        manager.delay(transfer, 200);

        manager.task(transfer, ()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
        });

        manager.delay(transfer, 200);

       /*manager.task(transfer, () -> {
           extendo.setSlidePosition(1);
           outake.scorePosLeft();
           lift.setLiftPosition(highChamberPos);
       });*/

       if(side == "right") {
            manager.task(transfer, () -> {
                extendo.setSlidePosition(1);
                outake.scorePosLeft();
                liftPosition = highChamberPos;
            });
        }

       if(side == "left"){
           manager.task(transfer, () -> {
               extendo.setSlidePosition(1);
               outake.scorePosMid();
               liftPosition = highBucketPos;
           });
        }

       manager.delay(transfer, 200);

       manager.forkTask(transfer, movement);
    }

    public void transferSequenceWhileScore(int extraX){
        TaskList scoring = new TaskList();

        manager.waypointTask(scoring, new Pose(0, -300, Math.toRadians(0)),0.8,0.5,150,false);

        manager.delay(scoring, 800);

        manager.waypointTask(scoring, new Pose(-100+extraX, -800, Math.toRadians(0)),0.4,0.5,150,false);

        manager.delay(scoring, 300);

        manager.task(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        manager.waypointTask(scoring, new Pose(-300, -400, 0),0.8,0.8,150,false);

        manager.task(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        TaskList transfer = new TaskList();

        intake.setIntakeState(UP);
        lift.setLiftState(IN);


        manager.task(transfer, ()->{
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        manager.delay(transfer, 600);

        manager.task(transfer, ()->{outake.transferPos();});

        manager.delay(transfer, 200);

        manager.task(transfer, ()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
        });

        manager.delay(transfer, 200);

       /*manager.task(transfer, () -> {
           extendo.setSlidePosition(1);
           outake.scorePosLeft();
           lift.setLiftPosition(highChamberPos);
       });*/

        if(side == "right") {
            manager.task(transfer, () -> {
                extendo.setSlidePosition(1);
                outake.scorePosLeft();
                liftPosition = highChamberPos;
            });
        }

        if(side == "left"){
            manager.task(transfer, () -> {
                extendo.setSlidePosition(1);
                outake.scorePosMid();
                liftPosition = highBucketPos;
            });
        }

        manager.delay(transfer, 200);

        manager.forkTask(transfer, scoring);
    }

}
