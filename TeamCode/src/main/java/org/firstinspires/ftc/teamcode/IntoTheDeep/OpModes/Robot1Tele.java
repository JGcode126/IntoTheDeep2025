package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;
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
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@TeleOp
@Config
public class Robot1Tele extends CuttleInitOpMode{
    double finalExtendoPos = 0;
    double finalLiftPos = 0;
    double counter = 0;
    public boolean transfering = false;
    public boolean autoPosing = false;
    double savex1,savey1, saver1;
    double savex2,savey2, saver2;
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
        encoderLocalizer.getPos().setR(Math.PI/2);
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
                extendoMotor.setPower(-0.5);
                rightBackSlides.setPower(-0.4);
                leftbackSlides.setPower(0.4);
                liftPosController.setHome();
                extendoPosController.setHome();
                finalExtendoPos = 0;
                finalLiftPos = 0;

            } else {
                if (!gamepad1.share) {
                    finalExtendoPos = extendo.extendoMachine(gamepad1.a, gamepad1.x, gamepad1.y, gamepad1.right_bumper, gamepad1.left_bumper);
                }
            }
            if (!gamepad1.share) {
                finalLiftPos = lift.liftMachine(gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.options, gamepad2.right_bumper, gamepad1.dpad_up, gamepad1.dpad_down);
            }
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
                outake.outakeMachine(gamepad2.a, false, false, false, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.share, gamepad2.left_bumper);
            }
        }


        if (!gamepad1.share) {
            liftPosition = finalLiftPos;
            extendoPosition = finalExtendoPos;
        }

        //dt.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        if (!autoPosing) {
            dt.driveDO(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger, -encoderLocalizer.getPos().getR());
        }

        if (autoPosing && gamepad1.right_stick_x != 0) {
            //dt.driveDO(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger, -encoderLocalizer.getPos().getR());
            queue.clear();
            autoPosing = false;
            dt.drive(0,0,0);
        }

        if (gamepad1.b){
            savex1 = encoderLocalizer.getPos().getX();
            savey1 = encoderLocalizer.getPos().getY();
            saver1 = encoderLocalizer.getPos().getR();
        }

        if (gamepad1.dpad_left && !autoPosing){
            autoPos();
        }

        if (gamepad1.dpad_up){
            savex2 = encoderLocalizer.getPos().getX();
            savey2 = encoderLocalizer.getPos().getY();
            saver2 = encoderLocalizer.getPos().getR();
        }

        if (gamepad1.dpad_right && !autoPosing){
            autoPos2();
        }


        if (intake.intakeState == TRANSFERED){
            transfering = true;
        }


        if(outake.outakeState == READY){
            finalLiftPos = 0;
            lift.setLiftState(IN);
        }



        if (gamepad1.options){
            encoderLocalizer.getPos().setR(0);
        }

        /*
        if(intake.getColor() == RED){
            intake.lightRed();
        }
        if(intake.getColor() == BLUE){
            intake.lightBlue();
        }
        if(intake.getColor() == YELLOW){
            intake.lightYellow();
        }

         */


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

    void frontScoreSequence(){
        TaskList frontScore = new TaskList();
        System.out.println("yes");
        lift.setLiftState(IN);
        outake.setScoreState(READY);
        frontScore.addTask(new CustomTask(()->{

            return true;
        }));
        frontScore.addTask(new DelayTask(400));
        frontScore.addTask(new CustomTask(()->{
            outake.readyPos();
            finalLiftPos = 0;
            dt.drive(0,0,0);
            return true;
        }));
        queue.addTask(frontScore);
    }

    void hardResetExtendo(){
        TaskList reset = new TaskList();
        extendo.setExtendoState(INE);
        reset.addTask(new CustomTask(()->{
            return extendoMotor.getCurrent() > 6000;
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

    void autoPos(){
        TaskList autoPos = new TaskList();
        System.out.println("yes");
        autoPosing = true;
        autoPos.addTask(new CustomTask(()->{
            //autoPosing = true;
            return true;
        }));
        autoPos.addTask(new PointTask(new Waypoint(new Pose(savex1,savey1,saver1),1, 0.4,75,false), ptpController));
        autoPos.addTask(new CustomTask(()->{
            autoPosing = false;
            return true;
        }));
        queue.addTask(autoPos);
    }

    void autoPos2(){
        TaskList autoPos2 = new TaskList();
        System.out.println("yes");
        autoPosing = true;
        autoPos2.addTask(new CustomTask(()->{
            //autoPosing = true;
            return true;
        }));
        autoPos2.addTask(new PointTask(new Waypoint(new Pose(savex2,savey2,saver2), 1, 0.4,75,false), ptpController));
        autoPos2.addTask(new CustomTask(()->{
            autoPosing = false;
            return true;
        }));
        queue.addTask(autoPos2);
    }



}
