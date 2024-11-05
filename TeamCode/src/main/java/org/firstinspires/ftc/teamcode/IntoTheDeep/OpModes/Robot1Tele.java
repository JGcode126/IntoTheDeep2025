package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.TRANSFERED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.TaskList;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@TeleOp
@Config
public class Robot1Tele extends CuttleInitOpMode{
    double finalExtendoPos = 0;
    double finalLiftPos = 0;
    double counter = 0;
    public boolean transfering = false;
    public void onInit() {
        super.onInit();

        intake.initPos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void main() {
        super.main();

        liftPosController.setHome();

    }

    public void mainLoop() {
        super.mainLoop();

        if (intake.intakeState == TRANSFERED && transfering == true){
            transferSequence();
            telemetry.addData("running", true);
        }

        if (transfering == false) {
            intake.intakeMachine(gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left);
            finalExtendoPos = extendo.extendoMachine(gamepad1.a, gamepad1.x, gamepad1.y);
            outake.outakeMachine(gamepad2.a, false, false, false, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right);
            finalLiftPos = lift.liftMachine(gamepad2.options, gamepad2.x, gamepad2.y, gamepad2.b, gamepad2.right_bumper);
        }

        liftPosition = finalLiftPos;
        extendoPosition = finalExtendoPos;

        dt.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        if (gamepad1.left_bumper && intake.intakeState == TRANSFERED){
            transfering = true;
        }

        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("intake state", intake.intakeState);
        telemetry.addData("outtake state", outake.outakeState);
        telemetry.addData("lift state", lift.LiftState);
        telemetry.addData("extendo pose", extendoPosController.getPosition());
        telemetry.addData("lift pose", liftPosController.getPosition());
        telemetry.addData("color detected", intake.getColor());
        telemetry.addData("transfering?", transfering);
        telemetry.addData("counter", counter);
        telemetry.update();
    }


    void transferSequence(){
        TaskList transfer = new TaskList();
        System.out.println("yes");
        intake.setIntakeState(UP);
        lift.setLiftState(IN);
        transfer.addTask(new CustomTask(()->{
            counter += 1;
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            finalExtendoPos = 0;
            finalLiftPos = 0;
            telemetry.addData("tranfer sequence running", true);
            return true;
        }));
        transfer.addTask(new DelayTask(300));
        transfer.addTask(new CustomTask(()->{
            //finalSlidePos = extendo.extendoMachine(true, false, false);
            outake.transferPos();
            return true;
        }));
        transfer.addTask(new DelayTask(100));
        transfer.addTask(new CustomTask(()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
            return true;
        }));
        transfer.addTask(new DelayTask(200));
        transfer.addTask(new CustomTask(()->{
            outake.scorePosMid();
            finalExtendoPos = 1;
            return true;
        }));
        transfer.addTask(new DelayTask(200));
        transfer.addTask(new CustomTask(()->{
            finalExtendoPos = 0;
            outake.setScoreState(BUCKET_BAR);
            extendo.setExtendoState(INE);

            transfering = false;
            return true;
        }));
        queue.addTask(transfer);
    }





}
