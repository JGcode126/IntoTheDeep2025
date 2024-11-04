package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.SECURED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.TRANSFERED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.GRIPPED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.HOLD;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.PLACED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.READY;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;

@TeleOp
@Config
public class Robot1Tele extends CuttleInitOpMode{
    double finalSlidePos = 0;
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
            transfering = false;
        }

        if (transfering == false) {
            intake.intakeMachine(gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left);
            outake.outakeMachine(gamepad2.a, false, false, false, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right);
            finalSlidePos = extendo.extendoMachine(gamepad1.a, gamepad1.x, gamepad1.y);
        }

        extendoPosition = finalSlidePos;

        dt.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        liftPosition = lift.liftMachine(gamepad2.options, gamepad2.x, gamepad2.y, gamepad2.b, gamepad2.right_bumper);

        if (gamepad2.left_bumper && intake.intakeState == TRANSFERED){
            transfering = true;
        }

        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("extendo", extendoPosController.getPosition());
        telemetry.addData("lift", liftPosController.getPosition());
        telemetry.addData("color", intake.getColor());
        telemetry.addData("intake state", intake.intakeState);
        telemetry.addData("outtake state", outake.outakeState);
        telemetry.addData("transfering", transfering);
        telemetry.addData("counter", counter);
        telemetry.update();
    }


    public void transferSequence (){
        queue.addTask(new CustomTask(()->{
            counter += 1;
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            intake.setIntakeState(UP);
            outake.readyPos();
            return true;
        }));
        queue.addTask(new DelayTask(300));
        queue.addTask(new CustomTask(()->{
            //finalSlidePos = extendo.extendoMachine(true, false, false);
            outake.transferPos();
            return true;
        }));
        queue.addTask(new DelayTask(100));
        queue.addTask(new CustomTask(()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
            return true;
        }));

        queue.addTask(new DelayTask(200));
        queue.addTask(new CustomTask(()->{
            outake.scorePosMid();
            finalSlidePos = 1;
            return true;
        }));
        queue.addTask(new DelayTask(200));
        queue.addTask(new CustomTask(()->{
            finalSlidePos = 0;
            intake.setIntakeState(UP);
            outake.setScoreState(BUCKET_BAR);
            transfering = false;
            return true;
        }));

    }


}
