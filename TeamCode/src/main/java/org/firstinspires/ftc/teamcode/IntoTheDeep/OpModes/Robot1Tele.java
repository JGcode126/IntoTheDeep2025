package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.TRANSFERED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BARLEFT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BARRIGHT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.READY;
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
        //intake.initPos();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void main() {
        super.main();
        liftPosController.setHome();
        extendoPosController.setHome();
        intake.initPos();
        outake.readyPos();
    }
    public void mainLoop() {
        super.mainLoop();

        if (intake.intakeState == TRANSFERED && transfering == true){
            teleOptransferSequence();
            telemetry.addData("running", true);
        }

        if (transfering == false) {
            intake.intakeMachine(gamepad2.dpad_down, gamepad2.right_trigger, gamepad2.dpad_up, gamepad2.left_trigger, gamepad2.right_stick_x);
            if(gamepad1.share){
                hardResetExtendo();
            } else {
                finalExtendoPos = extendo.extendoMachine(gamepad1.a, gamepad1.x, gamepad1.y, gamepad1.right_bumper, gamepad1.left_bumper);
            }
            finalLiftPos = lift.liftMachine(gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.options, gamepad2.right_bumper, gamepad1.dpad_up, gamepad1.dpad_down);

            if (outake.outakeState == BARLEFT || outake.outakeState == BARRIGHT){
                if(outake.outakeState == BARLEFT){
                    outake.scorePosLeft();
                    if (gamepad2.dpad_right){outake.setScoreState(BARRIGHT);}
                }
                if(outake.outakeState == BARRIGHT){
                    outake.scorePosRight();
                    if (gamepad2.dpad_left){outake.setScoreState(BARLEFT);}
                }
                if (gamepad2.a){
                    specimenDropOffSequence();
                }
            } else{
                outake.outakeMachine(gamepad2.a, false, false, false, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right);
            }
        }



        liftPosition = finalLiftPos;
        extendoPosition = finalExtendoPos;

        //dt.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        dt.driveDO(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger, -encoderLocalizer.getPos().getR());

        if (/*gamepad2.left_bumper &&*/ intake.intakeState == TRANSFERED){
            transfering = true;
        }

        if(outake.outakeState == READY){
            finalLiftPos = 0;
            lift.setLiftState(IN);
        }

        if (gamepad1.options){
            encoderLocalizer.getPos().setR(0);
        }

        if(gamepad1.share){
            hardResetExtendo();
        }


        telemetry.addData("intake state", intake.intakeState);
        telemetry.addData("outtake state", outake.outakeState);
        telemetry.addData("lift state", lift.currentState);
        telemetry.addData("extendo pose", extendoPosController.getPosition());
        telemetry.addData("lift pose", liftPosController.getPosition());
        telemetry.addData("turntable pose", intake.turntable.getPosition());
        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("transfering?", transfering);
        telemetry.update();
    }


    void teleOptransferSequence(){
        TaskList transfer = new TaskList();
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
        transfer.addTask(new DelayTask(600));
        transfer.addTask(new CustomTask(()->{
            //finalSlidePos = extendo.extendoMachine(true, false, false);
            outake.transferPos();
            return true;
        }));
        transfer.addTask(new DelayTask(200));
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

    void specimenDropOffSequence(){
        TaskList deliver = new TaskList();
        System.out.println("yes");
        lift.setLiftState(IN);
        outake.setScoreState(READY);
        deliver.addTask(new CustomTask(()->{
            outake.wristCenter();
            outake.openClaw();
            return true;
        }));
        deliver.addTask(new DelayTask(400));
        deliver.addTask(new CustomTask(()->{
            outake.readyPos();
            finalLiftPos = 0;
            dt.drive(0,0,0);
            return true;
        }));
        queue.addTask(deliver);
    }

    void hardResetExtendo(){
        TaskList reset = new TaskList();
        extendo.setExtendoState(INE);
        reset.addTask(new CustomTask(()->{
            extendo.hardRetract();
            return extendoMotor.getCurrent() > 3300;
        }));
        reset.addTask(new CustomTask(()->{
            extendoMotor.setPower(0);
            extendoPosController.setHome();
            return true;
        }));
        queue.addTask(reset);
    }

    void hardResetlift(){
        TaskList liftReset = new TaskList();
        lift.setLiftState(IN);
        liftReset.addTask(new CustomTask(()->{
            lift.hardReset();
            return leftbackSlides.getCurrent() > 3300;
        }));
        liftReset.addTask(new CustomTask(()->{
            leftbackSlides.setPower(0);
            rightBackSlides.setPower(0);
            liftPosController.setHome();
            return true;
        }));
        queue.addTask(liftReset);
    }





}
