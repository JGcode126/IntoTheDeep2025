package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;


public class RegularlyUsedSpecimenAuto extends CuttleInitOpMode{
    public boolean test = false;
    double counter = 0;
    public boolean transfering = false;
    double highChamberPos = 5.1;
    double highBucketPos = 13;

    public String color;
    public String side;

    public double liftSpecimen = 0;
    public int rotation = 0;
    ElapsedTime autoTimer;

    public RegularlyUsedSpecimenAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, CuttleIntake intake, CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
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

    //Init values for the start of teleOp
    public void park(){
        TaskList park = new TaskList();

        addWaypointTask(park, new Pose(-700, -250, Math.toRadians(90)),0.8,0.5,150,false);

        addIntakeTask(park, () -> {
            outake.readyPos();
            intake.off();
            intake.initPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        queue.addTask(park);
    }
    public void teleOpInit() {
        TaskList init = new TaskList();

        //init outake, intake, slides
        addIntakeTask(init, () -> {
            outake.readyPos();
            intake.off();
            intake.initPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        if(test) {
            addWaypointTask(init, new Pose(0, 0, 0),0.8,0.5,150,false);//for auto testing sets position of robot to 0
        }

        queue.addTask(init);
    }

    public void scoringSpecimen(int barpos){
        specimen();
        transferSequence();
        scoring(barpos);
    }

    public void specimen() {
        TaskList specimen = new TaskList();

        autoTimer.reset();
        specimen.addTask(new CustomTask(() -> {
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 2;
            liftPosition = 0;
            return true;
        }));

        //x = -300, y = -500, rotation = 45
        addWaypointTask(specimen, new Pose(-600, -400, Math.toRadians(45)),0.8,0.1,10,false);
        //addWaypointTask(specimen, new Pose(-350, -400, Math.toRadians(50)));
        /*
        addIntakeTask(specimen, () -> {
            intake.intakeDown();
            intake.in();
        });

         */
        specimen.addTask(new CustomTask(() -> {
            extendoPosition = 5;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE;
        }));

        //addDelayTask(specimen, 500);

        addIntakeTask(specimen, () -> {
            extendoPosition = 5;
            intake.clawClose();
        });

        //addDelayTask(specimen,1200);

        queue.addTask(specimen);
    }

    public void scoring(int barPos) {
        TaskList scoring = new TaskList();

        //addDelayTask(scoring, 500);


        //x = -150, y = -720, turn  = 0
        addWaypointTask(scoring, new Pose(0, -425, Math.toRadians(0)),0.8,0.5,150,true);
        addWaypointTask(scoring, new Pose(0, -650, Math.toRadians(0)),0.8,0.5,150,false);
        //addWaypointTask(scoring, new Pose(-150, -720, Math.toRadians(0)));


        addIntakeTask(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        addDelayTask(scoring, 300);
        //0
        addWaypointTask(scoring, new Pose(barPos, -400, 0),0.8,0.5,150,false);

        addIntakeTask(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        queue.addTask(scoring);
    }

    public void transferSequence(){
        TaskList transfer = new TaskList();
        TaskList movement = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        addWaypointTask(movement, new Pose(0, -100, Math.toRadians(0)),0.8,0.5,150,false);

        addIntakeTask(transfer, ()->{
            counter += 1;
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

        addIntakeTask(transfer, ()->{
            outake.scorePosMid();
            extendoPosition = 1;
        });

        addDelayTask(transfer, 200);

        if(side == "right") {
            addIntakeTask(transfer, () -> {
                extendoPosition = 0;
                outake.scorePosLeft();
                liftPosition = highChamberPos;

                transfering = false;
            });
        }

        else{
            addIntakeTask(transfer, () -> {
                extendoPosition = 0;
                outake.scorePosMid();
                liftPosition = highBucketPos;

                transfering = false;
            });
        }

        queue.addTask(new ForkTask(transfer, movement));

    }

    //--------------------------FOR SAMPLE DRIVING--------------------------

    public void sample(){
        fistSample();
        secondSample();
        thridSample();
    }

    //sample driving
    public void fistSample() {sampleDriving(-700, -400, -750, -1500, -1100, -1500, -1100, -300);}
    public void secondSample(){sampleDriving(-900, -1450,-1400, -1400,-1300, -300);}
    public void thridSample(){
        sampleDriving(-1250, -1450,-1500, -1400,-1480, -300);
        drive(-1400, -500, Math.PI);
    }


    public void ttSample(){
        fistSampleTT();
        secondSampleTT();
        thridSampleTT();
    }

    public void fistSampleTT() {
        ttdriving(-810, -590,90);
    }
    public void secondSampleTT(){
        ttdriving(-1055, -580,90);
    }
    public void thridSampleTT(){
        ttdriving(-1260, -610,100);
    }

    public void testSampleTT(int x, int y, int rotation, int x2, int y2, int r2){
        ttTestdriving(x, y,rotation, x2, y2, r2);
    }

    public void ttTestdriving(int x1, int y1, int r1,int x2, int y2, int r2) {
        //gets given positions to drive the robot

        TaskList sample = new TaskList();

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 4;
            intake.turntableRight();
            intake.in();
            return true;
        }));
        //make these slower
        addWaypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.75, 0.1, 10, false);
        //addWaypointTask(sample, new Pose(x1, y1, Math.toRadians(120)), 0.4, 0.1, 10, false);

        //addDelayTask(sample, 600);
        sample.addTask(new CustomTask(() -> {
            intake.in();
            boolean quit = false;
            if (encoderLocalizer.getPos().getR() < Math.toRadians(145)) {
                dt.drive(0, 0.1, -0.25);
            } else {
                dt.drive(0, 0, 0);
                quit = true;
            }
            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 4;
            intake.turntableLeft();
            return true;
        }));

        //x2 = -900, y2 = -400, r2 = 50
        addWaypointTask(sample, new Pose(x2, y2, Math.toRadians(r2)), 1, 0.5, 100, false);


        sample.addTask(new CustomTask(() -> {
            intake.out();
            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));
        queue.addTask(sample);
    }

    public void ttdriving(int x1, int y1, int r1) {
        //gets given positions to drive the robot

        TaskList sample = new TaskList();

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 4;
            intake.turntableRight();
            intake.in();
            return true;
        }));
        //make these slower
        addWaypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.75, 0.1, 10, false);
        //addWaypointTask(sample, new Pose(x1, y1, Math.toRadians(120)), 0.4, 0.1, 10, false);

        //addDelayTask(sample, 600);
        sample.addTask(new CustomTask(() -> {
            intake.in();
            boolean quit = false;
            if (encoderLocalizer.getPos().getR() < Math.toRadians(145)) {
                dt.drive(0, 0.1, -0.25);
            } else {
                dt.drive(0, 0, 0);
                quit = true;
            }
            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 4;
            intake.turntableLeft();
            return true;
        }));
        addWaypointTask(sample, new Pose(-900, -400, Math.toRadians(50)), 1, 0.5, 100, false);
        sample.addTask(new CustomTask(() -> {
            intake.out();
            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));
        queue.addTask(sample);
    }

    public void drive(int x, int y, double r){
        TaskList drive = new TaskList();

        addWaypointTask(drive, new Pose(x,y, r));

        queue.addTask(drive);
    }

    //driving robot to put specimen in observation zone
    public void sampleDriving(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
        //gets given positions to drive the robot

        TaskList sample = new TaskList();

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 0;
            return true;
        }));

        addWaypointTask(sample, new Pose(x1, y1, Math.PI), 1, 0.1, 200, true);
        addWaypointTask(sample, new Pose(x2, y2, Math.PI), 1, 0.01, 100, true);
        addWaypointTask(sample, new Pose(x3, y3, Math.PI), 1, 0.01, 100, true);

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 3;
            return true;
        }));

        addWaypointTask(sample, new Pose(x4, y4, Math.PI), 1, 0.01, 100, false);

        queue.addTask(sample);
    }

    public void sampleDriving(int x1, int y1, int x2, int y2, int x3, int y3) {
        //gets given positions to drive the robot

        TaskList sample = new TaskList();

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 0;
            return true;
        }));

        addWaypointTask(sample, new Pose(x1, y1, Math.PI), 1, 0.1, 200, true);
        addWaypointTask(sample, new Pose(x2, y2, Math.PI), 1, 0.01, 100, true);

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 3;
            return true;
        }));

        addWaypointTask(sample, new Pose(x3, y3, Math.PI), 1, 0.01, 100, false);

        queue.addTask(sample);
    }
    //----------------------------------------------------------------------


    //---------FOR SAMPLE INTAKING---------
    public void samplePos(int x, int y, double extIn, double extOut, double startR, double endR) {
        processSample(x, y, extIn, extOut, Math.toRadians(startR), Math.toRadians(endR));
        //x = x pos
        //y = y pos
        //extIn = extendo position for intaking
        //extOut = extendo posion for outake
        //startR = staring rotation
        //endR = ending rotation
    }

    public void processSample(int x, int y, double extendo, double extendoOut, double initialAngle, double finalAngle) {
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
    //----------------------------------------------------------------------

    //score first specimen during auto
    public void firstSpecimen() {
        TaskList scoringSpecimen = new TaskList();

        addIntakeTask(scoringSpecimen, () -> {
            liftPosition = 3;
            outake.autoAutoHighRungPos();
            extendoPosition = 3;
        });

        addWaypointTask(scoringSpecimen, new Pose(-100, -850, 0),0.6,0.5,150,false);
        addDelayTask(scoringSpecimen, 200);
        addIntakeTask(scoringSpecimen, () -> {
            outake.openClaw();
            liftPosition = 2.5;
            extendoPosition = 3;
        });

        addWaypointTask(scoringSpecimen, new Pose(-150, -400, Math.toRadians(50)),0.8,0.5,150,false);

        addIntakeTask(scoringSpecimen, () -> {
            outake.transferPos();
            liftPosition = 0;
            extendoPosition = 0;
            outake.readyPos();
        });

        queue.addTask(scoringSpecimen);
    }
}