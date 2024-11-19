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

@Autonomous
public class OptimizedSpecimenAuto extends CuttleInitOpMode {
    RegularlyUsed methods;
    private State currentState;
    double counter = 0;
    public boolean transfering = false;

    private ElapsedTime timer = new ElapsedTime();

    public void onInit() {
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
        addSequence(this::scoringSpecimen);
        addSequence(this::fistSample);
        addSequence(this::secondSample);
        addSequence(this::thirdSample);
        addSequence(this::transferSequence);
        addSequence(this::specimen);
        addSequence(this::teleOpInit);
    }

    public void mainLoop() {
        super.mainLoop();
        otosLocalizer.update();
        telemetryData();
    }

    private void telemetryData() {
        telemetry.addData("Intake Color", intake.getColor());
        telemetry.addData("Cuttle X:", otosLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:", otosLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:", otosLocalizer.getPos().getR());
        telemetry.addData("otos r", pos.h);
        telemetry.addData("otos x", pos.x);
        telemetry.addData("otos y", pos.y);
        telemetry.update();
    }

    private void addSequence(Runnable taskMethod) {
        queue.addTask(new CustomTask(() -> {
            taskMethod.run();
            return true;
        }));
    }

    private void addWaypointTask(TaskList taskList, Pose pose) {
        taskList.addTask(new PointTask(new Waypoint(pose), ptpController));
    }

    private void addDelayTask(TaskList taskList, int delay) {
        taskList.addTask(new DelayTask(delay));
    }

    private void addIntakeTask(TaskList taskList, Runnable intakeAction) {
        taskList.addTask(new CustomTask(() -> {
            intakeAction.run();
            return true;
        }));
    }

    void teleOpInit() {
        TaskList init = new TaskList();
        addIntakeTask(init, () -> {
            outake.readyPos();
            liftPosition = 0;
            extendoPosition = 0;
            intake.initPos();
        });
    }

    void specimen() {
        TaskList specimen = new TaskList();
        addIntakeTask(specimen, () -> {
            intake.intakeDown();
            extendoPosition = 7.5;
            intake.in();
        });
        addDelayTask(specimen, 1500);
        prepareForScoring(specimen);
    }

    private void prepareForScoring(TaskList taskList) {
        addDelayTask(taskList, 200);
        addIntakeTask(taskList, () -> {
            outake.autoHighRungPos();
            outake.wristLeft();
            liftPosition = 5.1;
        });
        addDelayTask(taskList, 200);
        addWaypointTask(taskList, new Pose(-200, -390, Math.toRadians(0)));
        addIntakeTask(taskList, () -> {
            outake.readyPos();
            liftPosition = 0;
        });
    }

    void transferSequence() {
        TaskList transfer = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        addIntakeTask(transfer, () -> {
            counter += 1;
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        addDelayTask(transfer, 300);
        addIntakeTask(transfer, outake::transferPos);
        addDelayTask(transfer, 200);
        addIntakeTask(transfer, () -> {
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
        });

        queue.addTask(transfer);
    }

    void thirdSample() {
        processSample(-1200, -580, 4.5, Math.toRadians(115), Math.toRadians(35));
    }

    void secondSample() {
        processSample(-900, -580, 7.5, Math.toRadians(125), Math.toRadians(45));
    }

    void fistSample() {
        processSample(-700, -600, 5.8, Math.toRadians(126), Math.toRadians(45));
    }

    private void processSample(int x, int y, double extendo, double initialAngle, double finalAngle) {
        TaskList sample = new TaskList();
        addWaypointTask(sample, new Pose(x, y, initialAngle));

        addIntakeTask(sample, () -> {
            intake.in();
            extendoPosition = extendo;
        });

        addDelayTask(sample, 1300);
        addWaypointTask(sample, new Pose(x, y, finalAngle));
        addIntakeTask(sample, () -> {
            intake.out();
            extendoPosition = 0;
        });
        addDelayTask(sample, 100);

        queue.addTask(sample);
    }

    void scoringSpecimen() {
        TaskList scoringSpecimen = new TaskList();
        addIntakeTask(scoringSpecimen, () -> {
            liftPosition = 3;
            outake.autoAutoHighRungPos();
        });

        addWaypointTask(scoringSpecimen, new Pose(0, -720, 0));
        addDelayTask(scoringSpecimen, 100);
        addIntakeTask(scoringSpecimen, () -> {
            outake.openClaw();
            liftPosition = 2.5;
        });

        addWaypointTask(scoringSpecimen, new Pose(0, -400, 0));
        addIntakeTask(scoringSpecimen, () -> {
            outake.transferPos();
            liftPosition = 0;
        });

        queue.addTask(scoringSpecimen);
    }

    private enum State {
        TO_SAMPLE,
        INTAKING_SAMPLE,
        SPIT_OUT_SAMPLE
    }
}
