package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos.Old;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.IntoTheDeep.Autos.RegularlyUsedSpecimenAuto;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
@Disabled
public class OptimizedSpecimenAuto extends CuttleInitOpMode {
    RegularlyUsedSpecimenAuto methods;
    private State currentState;
    double counter = 0;
    public boolean transfering = false;

    public static int x = -300;
    public static int y = -500;
    public static double rotation = 45;
    public static double extendoAuto = 7.5;
    public static double out = 7.5;

    double highChamberPos = 5.1;


    private ElapsedTime timer = new ElapsedTime();

    public void onInit() {
        super.onInit();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        otosLocalizer.reset();

        liftPosController.setHome();
        extendoPosController.setHome();

        intake.initPos();
        liftPosController.setHome();

        outake.initAutoPos();

        currentState = State.TO_SAMPLE;
    }

    public void main() {
        super.main();
        //----TESTING STUFF----
        //addSequence(this::test);
        //addSequence(this::testSample);

        //-----ACTUAL RUNNING CODE-----
        addSequence(this::firstSpecimen);
        //addSequence(this::fistSample);
        //addSequence(this::secondSample);
        //addSequence(this::thirdSample);

        addSequence(this::scoringSpecimen);
        addSequence(this::scoringSpecimen);
        addSequence(this::scoringSpecimen3);

        //addSequence(this::teleOpInit);
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
            intake.off();
            intake.initPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        addWaypointTask(init, new Pose(0,0,0));

        queue.addTask(init);
    }

    void scoringSpecimen3(){
        specimen();
        transferSequence();
        scoring(0.5);
    }

    void scoringSpecimen(){
        specimen();
        transferSequence();
        scoring(0);
    }

    void specimen() {
        TaskList specimen = new TaskList();

        //x = -300, y = -500, rotation = 45
        addWaypointTask(specimen, new Pose(-350, -600, Math.toRadians(50)));


        addIntakeTask(specimen, () -> {
            intake.intakeDown();
            intake.in();
        });

        addDelayTask(specimen, 500);

        addIntakeTask(specimen, () -> {
            extendoPosition = 7.5;
        });

        addDelayTask(specimen, 1500);

        queue.addTask(specimen);
    }

     void scoring(double num) {
        TaskList scoring = new TaskList();

         addIntakeTask(scoring, ()->{
             liftPosition = highChamberPos + num;
         });

         addWaypointTask(scoring, new Pose(-150, -720, Math.toRadians(0)));

         addIntakeTask(scoring, () -> {
             outake.openClaw();
             outake.wristCenter();
         });

         addDelayTask(scoring, 300);

         addWaypointTask(scoring, new Pose(0, -400, 0));

         addIntakeTask(scoring, () -> {
             outake.readyPos();
             liftPosition = 0;
         });

         queue.addTask(scoring);
     }

    void transferSequence(){
        TaskList transfer = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        addIntakeTask(transfer, ()->{
            counter += 1;
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
            telemetry.addData("tranfer sequence running", true);
        });

        addDelayTask(transfer, 300);

        addIntakeTask(transfer, ()->{
            //finalSlidePos = extendo.extendoMachine(true, false, false);
            outake.transferPos();
        });

        addDelayTask(transfer, 200);

        addIntakeTask(transfer, ()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
        });

        addDelayTask(transfer, 200);

        addIntakeTask(transfer, ()->{
            outake.scorePosMid();
            extendoPosition= 1;
        });

        addDelayTask(transfer, 200);

        addIntakeTask(transfer, ()->{
            extendoPosition = 0;
            outake.scorePosLeft();
            liftPosition = highChamberPos;

            transfering = false;
        });


        queue.addTask(transfer);
    }

    void thirdSample() {
        processSample(x, y, extendoAuto, out, Math.toRadians(rotation), Math.toRadians(45));//For testing
        //processSample(-1200, -580, 4.5, 4.5, Math.toRadians(115), Math.toRadians(35));
    }

    void secondSample() {
        processSample(x, y, extendoAuto, out, Math.toRadians(rotation), Math.toRadians(40));//For testing
        //processSample(-900, -580, 7.5, 3.5, Math.toRadians(120), Math.toRadians(45));

    }

    void fistSample() {
        //processSample(x, y, extendoAuto, out, Math.toRadians(rotation), Math.toRadians(45));//For Testing
        processSample(-700, -600, 5.7,6, Math.toRadians(130), Math.toRadians(47));
    }

    private void processSample(int x, int y, double extendo, double extendoOut, double initialAngle, double finalAngle) {
        TaskList sample = new TaskList();
        addWaypointTask(sample, new Pose(x, y, initialAngle));

        addIntakeTask(sample, () -> {
            intake.intakeDown();
            intake.in();
        });

        addDelayTask(sample, 200);

        addIntakeTask(sample, () -> {
            extendoPosition = extendo;
        });

        addDelayTask(sample, 1300);
        addIntakeTask(sample, () -> {
            extendoPosition = extendoOut;
        });
        addWaypointTask(sample, new Pose(x, y, finalAngle));
        addIntakeTask(sample, () -> {
            intake.out();
        });
        addDelayTask(sample, 200);

        addIntakeTask(sample, () -> {
            extendoPosition = 0;
        });

        queue.addTask(sample);
    }

    void firstSpecimen() {
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

    //--------TESTING CODE---------
    void test(){
        TaskList test = new TaskList();
        addDelayTask(test, 2000);

        addIntakeTask(test, () -> {
            outake.readyPos();
            //liftPosition = 0;
            extendoPosition = 7.5;
            intake.intakeDown();
        });

        addDelayTask(test, 2000);

        addIntakeTask(test, () -> {
            intake.initPos();
            extendoPosition = 0;
        });

        queue.addTask(test);
    }

    void testSample(){
        processSample(0, 0, 2, 0,0 , 0);
    }

    private enum State {
        TO_SAMPLE,
        INTAKING_SAMPLE,
        SPIT_OUT_SAMPLE
    }
}
