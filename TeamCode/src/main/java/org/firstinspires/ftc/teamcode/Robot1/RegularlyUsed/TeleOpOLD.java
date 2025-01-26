package org.firstinspires.ftc.teamcode.Robot1.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.Robot1.RegularlyUsed.SetupOLD.side;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleSlides.LiftState.IN;

import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleSlides;

public class TeleOpOLD extends CuttleInitOpMode{
    public double highChamberPos = 5;
    public double highBucketPos = 14;

    CuttleIntake intake;
    CuttleSlides lift;
    CuttleOutake outake;
    CuttleExtendo extendo;
    CuttleDT dt;
    TaskManagerOLD manager;

    public TeleOpOLD(CuttleIntake intake, CuttleOutake outake, CuttleExtendo extendo, CuttleSlides lift, CuttleDT dt, TaskManagerOLD manager) {

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

    public void justTransferSequence(){
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

        manager.task(transfer, () -> {
            //extendo.setSlidePosition(1);
            outake.scorePosLeft();
            liftPosition = highChamberPos;
        });

        manager.delay(transfer, 200);

        manager.addTask(transfer);
    }

    public void justTransferSequenceMore(int x, int y, int r, int x1, int y1, int r1, int x2, int y2, int r2){
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
        manager.waypointTask(transfer, new Pose(x, y, Math.toRadians(r)),0.8,0.8,150,false);


        manager.delay(transfer, 200);

        manager.task(transfer, ()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
        });

        manager.delay(transfer, 200);

        manager.task(transfer, () -> {
            outake.scorePosLeft();
            liftPosition = highChamberPos;
        });

        manager.delay(transfer, 200);

        manager.waypointTask(transfer, new Pose(x1, y1, Math.toRadians(r1)),0.8,0.6,150,false);

        manager.delay(transfer, 300);

        manager.task(transfer, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        manager.waypointTask(transfer, new Pose(x2, y2, Math.toRadians(r2)),0.8,0.8,150,false);

        manager.task(transfer, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        manager.addTask(transfer);
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

        manager.waypointTask(scoring, new Pose(0, -300, Math.toRadians(0)),0.8,0.6,150,false);

        manager.delay(scoring, 1000);

        manager.waypointTask(scoring, new Pose(-100+extraX, -760, Math.toRadians(0)),0.7,0.5,150,false);

        manager.delay(scoring, 300);

        manager.task(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        manager.waypointTask(scoring, new Pose(-300, -400, 0),1,0.8,150,false);

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

        //if(side == "right") {
            manager.task(transfer, () -> {
                extendoPosition = 1;
                outake.scorePosLeft();
                liftPosition = highChamberPos;
            });
        //}

        /*if(side == "left"){
            manager.task(transfer, () -> {
                extendo.setSlidePosition(1);
                outake.scorePosMid();
                liftPosition = highBucketPos;
            });
        }*/

        //manager.delay(transfer, 200);

        manager.forkTask(transfer, scoring);
    }

}
