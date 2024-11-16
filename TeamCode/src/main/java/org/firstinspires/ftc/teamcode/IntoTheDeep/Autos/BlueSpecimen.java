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
        otosLocalizer.reset();

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

        queue.addTask(new CustomTask(() ->{
            transferSequence();
            return true;
        }));
    }
    public void mainLoop(){
        super.mainLoop();

        otosLocalizer.update();
        System.out.println(otosLocalizer.getPos());
        telemetry.addData("Intake Color", intake.getColor());
        telemetry.addData("Cuttle X:",otosLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",otosLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",otosLocalizer.getPos().getR());
        telemetry.addData("otos r", pos.h);
        telemetry.addData("otos x", pos.x);
        telemetry.addData("otos y", pos.y);
        telemetry.update();
    }

    void transferSequence(){
        TaskList transfer = new TaskList();

        transfer.addTask(new CustomTask(()->{
            intake.intakeDown();
            extendoPosition = 7.5;
            intake.in();
            return true;
        }));

        transfer.addTask(new DelayTask(1500));

        intake.setIntakeState(UP);
        lift.setLiftState(IN);
        transfer.addTask(new CustomTask(()->{

            counter += 1;
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
            telemetry.addData("tranfer sequence running", true);
            return true;
        }));
        transfer.addTask(new DelayTask(300));
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
            outake.autoHighRungPos();
            outake.wristLeft();
            liftPosition = 5.1;
            return true;
        }));
        transfer.addTask(new DelayTask(200));
        transfer.addTask(new CustomTask(()->{
            intake.intakeDown();
            //extendoPosition = 7.5;
            transfering = false;
            return true;
        }));

        /*transfer.addTask(new CustomTask(()->{
            intake.intakeDown();
            extendoPosition = 7.5;
            intake.in();
            return true;
        }));*/

        transfer.addTask(new PointTask(new Waypoint(new Pose(-200, -390, Math.toRadians(0))), ptpController));

        transfer.addTask(new CustomTask(()->{
            outake.readyPos();
            liftPosition = 0;
            return true;
        }));

        queue.addTask(transfer);
    }

    void endIntaking() {
        TaskList endIntaking = new TaskList();

        /*if(intake.getColor() == CuttleIntake.Color.BLUE){
            endIntaking.addTask(new CustomTask(()->{
                detectingBlue(5);
                return true;
            }));
        }

        else{
            endIntaking.addTask(new DelayTask(200));
        }*/

        endIntaking.addTask(new DelayTask(1300));
        endIntaking.addTask(new CustomTask(()->{
            intake.off();
            return true;
        }));

        endIntaking.addTask(new PointTask(new Waypoint(new Pose(-700, -600, Math.toRadians(45))), ptpController));

        endIntaking.addTask(new CustomTask(()->{
            intake.out();
            return true;
        }));

        endIntaking.addTask(new DelayTask(100));

        endIntaking.addTask(new CustomTask(()->{
            intake.off();
            extendoPosition = 0;
            return true;
        }));

        endIntaking.addTask(new PointTask(new Waypoint(new Pose(-900, -580, Math.toRadians(125))), ptpController));

        endIntaking.addTask(new CustomTask(()->{
            intake.in();
            extendoPosition = 7.5;
            return true;
        }));

        endIntaking.addTask(new DelayTask(1300));

        endIntaking.addTask(new CustomTask(()->{
            intake.in();
            extendoPosition = 6;
            return true;
        }));

        endIntaking.addTask(new PointTask(new Waypoint(new Pose(-900, -580, Math.toRadians(45))), ptpController));

        endIntaking.addTask(new CustomTask(()->{
            intake.out();
            return true;
        }));

        endIntaking.addTask(new DelayTask(100));

        endIntaking.addTask(new CustomTask(()->{
            intake.off();
            extendoPosition = 0;
            return true;
        }));

        endIntaking.addTask(new PointTask(new Waypoint(new Pose(-1200, -580, Math.toRadians(115))), ptpController));

        endIntaking.addTask(new CustomTask(()->{
            intake.in();
            extendoPosition = 4.5;
            return true;
        }));

        endIntaking.addTask(new DelayTask(1300));

        endIntaking.addTask(new CustomTask(()->{
            extendoPosition = 2;
            return true;
        }));

        endIntaking.addTask(new PointTask(new Waypoint(new Pose(-1200, -580, Math.toRadians(35))), ptpController));

        endIntaking.addTask(new CustomTask(()->{
            extendoPosition = 6.5;
            intake.out();
            return true;
        }));

        endIntaking.addTask(new DelayTask(100));

        endIntaking.addTask(new CustomTask(()->{
            intake.off();
            extendoPosition = 0;
            return true;
        }));

        endIntaking.addTask(new PointTask(new Waypoint(new Pose(-350, -350, Math.toRadians(45))), ptpController));
        queue.addTask(endIntaking);

    }


    void intakingSample(){
        TaskList intakingSample = new TaskList();

        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-700, -600, Math.toRadians(126))), ptpController));


        intakingSample.addTask(new CustomTask(()->{
            outake.readyPos();
            intake.intakeDown();
            intake.in();
            return true;
        }));
        intakingSample.addTask(new DelayTask(500));

        intakingSample.addTask(new CustomTask(()->{
            extendoPosition = 5.8;
            return true;
        }));

        queue.addTask(intakingSample);
    }

    void scoringSpecimen(){
        TaskList scoringSpecimen = new TaskList();

        scoringSpecimen.addTask(new CustomTask(() -> {
            liftPosition = 3;
            outake.autoAutoHighRungPos();
            return true;
        }));

        scoringSpecimen.addTask(new PointTask(new Waypoint(new Pose(0, -720, 0)), ptpController));

        scoringSpecimen.addTask(new DelayTask(100));
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
