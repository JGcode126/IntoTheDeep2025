package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.Old;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.ForkTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;


public class RegularlyUsedBucketAuto extends CuttleInitOpMode{

    ElapsedTime autoTimer;

    public RegularlyUsedBucketAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, CuttleIntake intake, CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                                   PTPController ptpController, MotorPositionController liftController, MotorPositionController extController,
                                   CuttleExtendo extendo, CuttleSlides lift, CuttleDT dt) {

        //Initializing values
        this.otosLocalizer = otos;
        this.encoderLocalizer = encoderLocalizer;
        this.intake = intake;
        this.outake = outake;
        this.telemetry = telemetry;
        this.queue = queue;
        this.ptpController = ptpController;
        this.liftPosController = liftController;
        this.extendoPosController = extController;
        this.lift = lift;
        this.extendo = extendo;
        this.dt = dt;
        autoTimer = new ElapsedTime();
    }

    //To initialize the robot for start
    public void initRobot() {
        otosLocalizer.reset();//reset otos
        encoderLocalizer.reset();//reset odo

        //inti intake and outake
        outake.initAutoPos();
        intake.initPos();

        //set slide home
        liftPosController.setHome();
        extendoPosController.setHome();
    }

    //print out data during auto
    public void telemetryData() {
        //Printing out data
        //works!!
        otosLocalizer.update();//updating otos values
        Pose currentPos = otosLocalizer.getPos();//setting position of otos to current pos

        telemetry.addData("Intake Color", intake.getColor());//print out intake color

        //print out oto values
        telemetry.addData("Cuttle X:", currentPos.getX());
        telemetry.addData("Cuttle Y:", currentPos.getY());
        telemetry.addData("Cuttle R:", currentPos.getR());

        telemetry.update();
    }

    //I don't think this is needed
    public void addSequence(Runnable taskMethod) {
        queue.addTask(new CustomTask(() -> {
            taskMethod.run();
            return true;
        }));
    }

    //Add position task
    public void addWaypointTask(TaskList taskList, Pose pose) {
        //takes in the task and position
        //adds the position to the given taskList
        taskList.addTask(new PointTask(new Waypoint(pose) , ptpController));
    }

    //Constructer gets in other stuff too -- power, rSlop, tSlop, passthrough
    //More detailed driving
    //Made this to not mess up my code
    public void addWaypointTask(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        //takes in the task and position
        //adds the position to the given taskList
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), ptpController));
    }

    //To add a delay task
    public void addDelayTask(TaskList taskList, int delay) {
        //takes delay and taskList
        //adds delay to task list
        taskList.addTask(new DelayTask(delay));
        //idk if this is needed
    }

    //Intake task
    //Also idk if this is needed
    public void addIntakeTask(TaskList taskList, Runnable intakeAction) {
        taskList.addTask(new CustomTask(() -> {
            intakeAction.run();
            return true;
        }));
    }

    public void scoreFirstSample(int finishxpos) {
        TaskList scoringSample = new TaskList();

        addIntakeTask(scoringSample, () -> {
            liftPosition = 14;
            outake.scorePosMid();
        });
        //x used to be -980, -1000 still works
        addWaypointTask(scoringSample, new Pose(-980, 200, -0.7),0.8,0.6,10,false);


        //addWaypointTask(scoringSample, new Pose(-1200, 260, -0.7),0.8,0.5,150,false);

        addIntakeTask(scoringSample, () -> {
            outake.openClaw();
        });
        addDelayTask(scoringSample, 200);
        addIntakeTask(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });
        addWaypointTask(scoringSample, new Pose(finishxpos, 330, 0),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }

    public void scoreFirstSample(int xPos, int yPos, int finishxpos) {
        TaskList scoringSample = new TaskList();

        addIntakeTask(scoringSample, () -> {
            liftPosition = 14;
            outake.scorePosMid();
        });
        //x used to be -980, -1000 still works
        addWaypointTask(scoringSample, new Pose(xPos, yPos, -0.7),0.6,0.6,10,false);


        //addWaypointTask(scoringSample, new Pose(-1200, 260, -0.7),0.8,0.5,150,false);

        addIntakeTask(scoringSample, () -> {
            outake.openClaw();
        });
        addDelayTask(scoringSample, 200);
        addIntakeTask(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });
        addWaypointTask(scoringSample, new Pose(finishxpos, 330, 0),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }

    public void scoreSample(int finishxpos) {
        TaskList scoringSample = new TaskList();

        addIntakeTask(scoringSample, () -> {
            liftPosition = 14;
            outake.scorePosMid();
        });

        //change x from -920, y used to be 400
        addWaypointTask(scoringSample, new Pose(-1000, 390, -0.7),0.8,0.6,10,false);
        addIntakeTask(scoringSample, () -> {
            dt.drive(-0.2,0,0);
        });
        addDelayTask(scoringSample, 700);
        //addWaypointTask(scoringSample, new Pose(-1200, 260, -0.7),0.8,0.5,150,false);

        addIntakeTask(scoringSample, () -> {
            dt.drive(0,0,0);
            outake.openClaw();
        });
        addDelayTask(scoringSample, 200);
        addIntakeTask(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });
        addWaypointTask(scoringSample, new Pose(finishxpos, 330, 0),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }

    public void scoring(int xpos, int xfinishpos, int deg){
        intakeSample(xpos, deg);
        transferSequence();
        scoreSample(xpos);
    }

    public void park(){
        TaskList park = new TaskList();
        park.addTask(new CustomTask(() -> {
            intake.turntableMiddle();
            intake.armUp();
            extendoPosition = 0;
            liftPosition = 0;
            outake.parkPos();
            return true;
        }));

        addWaypointTask(park, new Pose(-500, 1400, Math.toRadians(90)),0.9,0.5,100,false);
        park.addTask(new CustomTask(() -> {
            dt.drive(-0.5,0,0);
            return true;
        }));
        queue.addTask(park);
    }

    public void intakeSample(double x, double deg) {
        TaskList samples = new TaskList();

        samples.addTask(new CustomTask(() -> {
            autoTimer.reset();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 0;
            liftPosition = 0;
            return true;
        }));

        //x = -300, y = -500, rotation = 45
        addWaypointTask(samples, new Pose(x, 330, Math.toRadians(deg)),0.9,0.1,10,false);


        samples.addTask(new DelayTask(400));
        samples.addTask(new CustomTask(() -> {
            boolean quit = false;
            extendoPosition = 5;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            if (autoTimer.seconds() > 3.5) {quit = true;}

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        //addDelayTask(specimen, 500);

        addIntakeTask(samples, () -> {
            extendoPosition = 0;
            intake.clawClose();
        });

        //addDelayTask(specimen,1200);

        queue.addTask(samples);
    }

    public void transferSequence(){
        TaskList transfer = new TaskList();
        TaskList movement = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        //used to be y = -180
        addWaypointTask(movement, new Pose(-920, 400, -0.7),0.4,0.1,10,false);

        addIntakeTask(transfer, ()->{
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            extendoPosition = 0;
            liftPosition = 0;
            telemetry.addData("tranfer sequence running", true);
        });

        addDelayTask(transfer, 600);

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

        addIntakeTask(transfer, () -> {
            extendoPosition = 1;
            outake.scorePosLeft();
            liftPosition = 14;


        });

        /*if(side == "right") {
            addIntakeTask(transfer, () -> {
                extendoPosition = 1;
                outake.scorePosLeft();
                liftPosition = highChamberPos;

                transfering = false;
            });
        }

        else{
            addIntakeTask(transfer, () -> {
                extendoPosition = 1;
                outake.scorePosMid();
                liftPosition = highBucketPos;

                transfering = false;
            });
        }*/

        addDelayTask(transfer, 200);

        queue.addTask(new ForkTask(transfer, movement));

    }


}