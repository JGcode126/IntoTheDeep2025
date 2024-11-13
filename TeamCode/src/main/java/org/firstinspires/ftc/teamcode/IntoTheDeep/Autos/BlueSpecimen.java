package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;

@Autonomous
public class BlueSpecimen extends CuttleInitOpMode {
    RegularlyUsed methods;
    private State currentState;
    double finalExtendoPos = 0;
    double finalLiftPos = 0;
    double counter = 0;
    public boolean transfering = false;

    private ElapsedTime timer = new ElapsedTime();

    public void onInit(){
        super.onInit();
        methods = new RegularlyUsed();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        encoderLocalizer.reset();

        liftPosition = 0;
        extendoPosition = 0;

        intake.initPos();
        liftPosController.setHome();

        outake.closeClaw();
        outake.autoPos();

        currentState = State.TO_SAMPLE;
    }
    public void main() {
        super.main();
        queue.addTask(new CustomTask(() -> {
            scoringSpecimen();
            return true;
        }));

        queue.addTask(new CustomTask(() -> {
            intakingSample();
            return true;
        }));

        queue.addTask(new CustomTask(() -> {
            endIntaking();
            return true;
        }));
    }
    public void mainLoop(){
        super.mainLoop();

        encoderLocalizer.update();
        System.out.println(encoderLocalizer.getPos());
        telemetry.addData("Intake Color", intake.getColor());
        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("otos r", pos.h);
        telemetry.addData("otos x", pos.x);
        telemetry.addData("otos y", pos.y);
        telemetry.update();
    }

    void transferSequence(){
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

    void endIntaking() {
        TaskList endIntaking = new TaskList();

        if(intake.getColor() == CuttleIntake.Color.BLUE){
            endIntaking.addTask(new CustomTask(()->{
                detectingBlue(5);
                return true;
            }));
        }

        else{
            endIntaking.addTask(new DelayTask(200));
        }

        queue.addTask(endIntaking);
    }

    public void detectingBlue(double time){
        timer.reset();
        while (time > timer.seconds()) {
            if (intake.getColor() == CuttleIntake.Color.BLUE) {
                intake.off();
            }
            else{
                intake.in();
            }
        }
    }


    void intakingSample(){
        TaskList intakingSample = new TaskList();

        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-550, -500, Math.toRadians(128))), ptpController));


        intakingSample.addTask(new CustomTask(()->{
            intake.intakePos(0.5);
            intake.in();
            return true;
        }));
        intakingSample.addTask(new DelayTask(500));

        intakingSample.addTask(new CustomTask(()->{
            extendoPosition = 7.5;
            return true;
        }));

        queue.addTask(intakingSample);
    }

    void scoringSpecimen(){
        TaskList scoringSpecimen = new TaskList();

        scoringSpecimen.addTask(new CustomTask(() -> {
            liftPosition = 3;
            outake.autoHighRungPos();
            return true;
        }));

        scoringSpecimen.addTask(new PointTask(new Waypoint(new Pose(0, -750, 0)), ptpController));

        scoringSpecimen.addTask(new DelayTask(500));
        scoringSpecimen.addTask(new CustomTask(() -> {
            outake.openClaw();
            liftPosition = 2.5;
            return true;
        }));

        scoringSpecimen.addTask(new PointTask(new Waypoint(new Pose(0, -400, 0)), ptpController));

        scoringSpecimen.addTask(new CustomTask(() -> {
            outake.transferPos();
            liftPosition = 0;
            return true;
        }));

        queue.addTask(scoringSpecimen);
    }

    private enum State{
        TO_SAMPLE,
        INTAKING_SAMPLE,
        SPIT_OUT_SAMPLE
    }
}
