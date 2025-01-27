package org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides.LiftState.IN;

import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleDT;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides;

public class TeleOp extends CuttleInitOpModeRobot2 {
    public double highChamberPos = 5;
    public double highBucketPos = 14;

    v2CuttleIntake intake;
    v2CuttleSlides lift;
    v2CuttleOutake outake;
    v2CuttleExtendo extendo;
    v2CuttleDT dt;
    TaskManager manager;

    public TeleOp(v2CuttleIntake intake, v2CuttleOutake outake, v2CuttleExtendo extendo,
                  v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager) {

        this.intake = intake;
        this.lift = lift;
        this.outake = outake;
        this.extendo = extendo;
        this.dt = dt;
        this.manager = manager;

    }

    public void teleOptransferSequence(double extraX){
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

        manager.task(transfer, () -> {
            intake.setIntakeState(UP);
            lift.setLiftState(IN);
        });

        manager.task(transfer, () ->{
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
            telemetry.addData("tranfer sequence running", true);
        });

        manager.delay(transfer, 600);

        manager.task(transfer, () ->{
            outake.transferPos();
        });

        manager.delay(transfer,200);

        manager.task(transfer, () ->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
        });

        manager.delay(transfer,200);

        manager.task(transfer, () ->{
            outake.scorePosMid();
            extendoPosition = 1;
        });

        manager.delay(transfer,200);

        manager.task(transfer, () ->{
            extendoPosition = 0;
            outake.setScoreState(BUCKET_BAR);
            extendo.setExtendoState(INE);
        });
        
        manager.forkTask(transfer,scoring);
    }

}
