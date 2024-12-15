package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
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

public class TeleOp extends CuttleInitOpMode {
    double counter = 0;
    public boolean transfering = false;
    public double highChamberPos = 5.1;
    double highBucketPos = 13;
    private String side = setup.side;

    public void transferSequence(){
        TaskList transfer = new TaskList();
        TaskList movement = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        task.addWaypointTask(movement, new Pose(0, -100, Math.toRadians(0)),0.8,0.5,150,false);

        task.addIntakeTask(transfer, ()->{
            counter += 1;
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
            telemetry.addData("tranfer sequence running", true);
        });

        task.addDelayTask(transfer, 600);

        task.addIntakeTask(transfer, ()->{
            //finalSlidePos = extendo.extendoMachine(true, false, false);
            outake.transferPos();
        });

        task.addDelayTask(transfer, 200);

        task.addIntakeTask(transfer, ()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
        });

        task.addDelayTask(transfer, 200);

        if(side == "right") {
            task.addIntakeTask(transfer, () -> {
                extendoPosition = 1;
                outake.scorePosLeft();
                liftPosition = highChamberPos;

                transfering = false;
            });
        }

        else if(side == "left"){
            task.addIntakeTask(transfer, () -> {
                extendoPosition = 1;
                outake.scorePosMid();
                liftPosition = highBucketPos;

                transfering = false;
            });
        }

        else{

        }

        task.addDelayTask(transfer, 200);

        queue.addTask(new ForkTask(transfer, movement));

    }
}
