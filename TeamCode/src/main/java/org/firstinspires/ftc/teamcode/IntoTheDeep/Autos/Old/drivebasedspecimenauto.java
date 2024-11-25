package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos.Old;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.READY;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.opencv.core.Mat;

@Autonomous
@Disabled
public class drivebasedspecimenauto extends CuttleInitOpMode {
    private State currentState;
    double counter = 0;
    public boolean transfering = false;
    boolean step = false;
    boolean dropoff = false;
    private ElapsedTime timer = new ElapsedTime();

    public void onInit(){
        super.onInit();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        otosLocalizer.reset();

        liftPosition = 0;
        extendoPosition = 0;

        intake.initPos();
        liftPosController.setHome();

        outake.initAutoPos();

        currentState = State.TO_SAMPLE;
    }
    public void main() {
        super.main();
        queue.addTask(new CustomTask(() -> {
            scoringSpecimen();
            return true;
        }));
        queue.addTask(new CustomTask(() -> {
            fistSample();
            return true;
        }));
        /*
        queue.addTask(new CustomTask(()->{
            intake.intakeDown();
            extendoPosition = 6;
            intake.in();
            return true;
        }));
        queue.addTask(new DelayTask(1500));

        queue.addTask(new CustomTask(()->{
            transferSequence();
            return transferSequence();
        }));

        queue.addTask(new PointTask(new Waypoint(new Pose(-200, -390, Math.toRadians(0))), ptpController));

        queue.addTask(new CustomTask(()->{
            liftPosition = 5.1;
            return true;
        }));
        queue.addTask(new PointTask(new Waypoint(new Pose(0, -720, 0),1,0.010,30,false), ptpController));

        queue.addTask(new CustomTask(()->{
            specimenDropOffSequence();
            return specimenDropOffSequence();
        }));
        queue.addTask(new PointTask(new Waypoint(new Pose(-350, -350, Math.toRadians(45))), ptpController));

         */


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

    void teleOpInit(){
        TaskList init = new TaskList();

        init.addTask(new CustomTask(()->{
            outake.readyPos();
            liftPosition = 0;
            extendoPosition = 0;
            intake.initPos();
            return true;
        }));
    }

    void specimen(){
        TaskList specimen = new TaskList();

        specimen.addTask(new CustomTask(()->{
            intake.intakeDown();
            extendoPosition = 7.5;
            intake.in();
            return true;
        }));

        specimen.addTask(new DelayTask(1500));
        //scoring

        specimen.addTask(new DelayTask(200));
        specimen.addTask(new CustomTask(()->{
            transferSequence();
            return true;
        }));

        specimen.addTask(new DelayTask(200));
        specimen.addTask(new CustomTask(()->{
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

        specimen.addTask(new PointTask(new Waypoint(new Pose(-200, -390, Math.toRadians(0))), ptpController));

        specimen.addTask(new CustomTask(()->{
            liftPosition = 5.1;
            return true;
        }));
        specimen.addTask(new PointTask(new Waypoint(new Pose(0, -720, 0),1,0.010,30,false), ptpController));

        specimen.addTask(new CustomTask(()->{
            specimenDropOffSequence();
            return true;
        }));
        specimen.addTask(new PointTask(new Waypoint(new Pose(-350, -350, Math.toRadians(45))), ptpController));

        queue.addTask(specimen);

    }

    boolean transferSequence(){
        TaskList transfer = new TaskList();

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
            outake.transferPos();
            return true;
        }));

        transfer.addTask(new DelayTask(200));
        transfer.addTask(new CustomTask(()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
            transfering = true;
            return true;
        }));

        queue.addTask(transfer);
        return transfering;
    }

    boolean specimenDropOffSequence(){
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
            liftPosition = 0;
            dt.drive(0,0,0);
            dropoff = true;
            return true;
        }));
        queue.addTask(deliver);
        return dropoff;
    }


    void fistSample(){
        TaskList intakingSample = new TaskList();
        //-------------------------
        intakingSample.addTask(new CustomTask(()->{
            extendoPosition = 0;
            return true;
        }));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-900, -1450, Math.PI),1,0.010,100,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1300, -1400, Math.PI),1,0.010,100,true), ptpController));
        intakingSample.addTask(new CustomTask(()->{
            extendoPosition = 3;
            return true;
        }));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1300, -200, Math.PI),0.7,0.010,100,true), ptpController));
        //-----------------------

        intakingSample.addTask(new CustomTask(()->{
            extendoPosition = 0;
            return true;
        }));

        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1250, -1450, Math.PI),1,0.010,100,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1470, -1400, Math.PI),1,0.010,75,true), ptpController));
        intakingSample.addTask(new CustomTask(()->{
            extendoPosition = 3;
            return true;
        }));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1470, -200, Math.PI),0.7,0.010,100,true), ptpController));



        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-400, -500, Math.PI/2),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-350, -350, Math.toRadians(45))), ptpController));


        queue.addTask(intakingSample);
        intakingSample.addTask(new CustomTask(()->{
            step = true;
            return true;
        }));
    }

    void scoringSpecimen(){
        TaskList scoringSpecimen = new TaskList();

        scoringSpecimen.addTask(new CustomTask(() -> {
            liftPosition = 3;
            extendoPosition = 3;
            outake.autoAutoHighRungPos();
            return true;
        }));

        scoringSpecimen.addTask(new PointTask(new Waypoint(new Pose(0, -720, 0),1,0.010,30,false), ptpController));

        scoringSpecimen.addTask(new DelayTask(100));
        scoringSpecimen.addTask(new CustomTask(() -> {
            outake.openClaw();
            liftPosition = 2.5;
            return true;
        }));

        scoringSpecimen.addTask(new PointTask(new Waypoint(new Pose(0, -400, 0),1,0.010,100, true), ptpController));

        scoringSpecimen.addTask(new CustomTask(() -> {
            outake.readyPos();
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
