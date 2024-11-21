package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
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

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.opencv.core.Mat;

@Autonomous
public class drivebasedspecimenauto extends CuttleInitOpMode {
    RegularlyUsed methods;
    private State currentState;
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
            fistSample();
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
            outake.autoHighRungPos();
            outake.wristLeft();
            liftPosition = 5.1;
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
            outake.readyPos();
            liftPosition = 0;
            return true;
        }));

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
            return true;
        }));

        queue.addTask(transfer);
    }



    void thirdSample() {
        TaskList third = new TaskList();

        third.addTask(new PointTask(new Waypoint(new Pose(-1200, -580, Math.toRadians(115))), ptpController));

        third.addTask(new CustomTask(()->{
            intake.in();
            extendoPosition = 4.5;
            return true;
        }));

        third.addTask(new DelayTask(1300));

        third.addTask(new CustomTask(()->{
            extendoPosition = 2;
            return true;
        }));

        third.addTask(new PointTask(new Waypoint(new Pose(-1200, -580, Math.toRadians(35))), ptpController));

        third.addTask(new CustomTask(()->{
            extendoPosition = 6.5;
            intake.out();
            return true;
        }));

        third.addTask(new DelayTask(100));

        third.addTask(new CustomTask(()->{
            intake.off();
            extendoPosition = 0;
            return true;
        }));

        third.addTask(new PointTask(new Waypoint(new Pose(-350, -350, Math.toRadians(45))), ptpController));
        queue.addTask(third);
    }

    void secondSample(){
        TaskList second = new TaskList();

        second.addTask(new PointTask(new Waypoint(new Pose(-900, -580, Math.toRadians(125))), ptpController));

        second.addTask(new CustomTask(()->{
            intake.in();
            extendoPosition = 7.5;
            return true;
        }));

        second.addTask(new DelayTask(1300));

        second.addTask(new CustomTask(()->{
            intake.in();
            extendoPosition = 6;
            return true;
        }));

        second.addTask(new PointTask(new Waypoint(new Pose(-900, -580, Math.toRadians(45))), ptpController));

        second.addTask(new CustomTask(()->{
            intake.out();
            return true;
        }));

        second.addTask(new DelayTask(100));

        second.addTask(new CustomTask(()->{
            intake.off();
            extendoPosition = 0;
            return true;
        }));
    }


    void fistSample(){
        TaskList intakingSample = new TaskList();
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-800, -400, 0),1,0.010,200,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-900, -1500, Math.PI),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1250, -1500, Math.PI),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1250, -400, Math.PI),1,0.010,150,true), ptpController));

        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1250, -1450, Math.PI),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1500, -1400, Math.PI),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1500, -400, Math.PI),1,0.010,150,true), ptpController));

        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1500, -1450, Math.PI),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1650, -1400, Math.PI),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1650, -400, Math.PI),1,0.010,150,true), ptpController));

        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-1000, -600, Math.PI/2),1,0.010,150,true), ptpController));
        intakingSample.addTask(new PointTask(new Waypoint(new Pose(-400, -500, Math.PI/2),1,0.010,150,false), ptpController));

        queue.addTask(intakingSample);
    }

    void scoringSpecimen(){
        TaskList scoringSpecimen = new TaskList();

        scoringSpecimen.addTask(new CustomTask(() -> {
            liftPosition = 3;
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
