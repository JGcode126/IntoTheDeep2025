package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.Old;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.LOOKING;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.SECURED;
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

import java.util.concurrent.atomic.AtomicBoolean;


public class RegularlyUsedSpecimenAuto extends CuttleInitOpMode{
    public boolean test = false;
    double counter = 0;
    public boolean transfering = false;
    public double highChamberPos = 5;
    double highBucketPos = 13;

    public String color;
    public String side;

    public double liftSpecimen = 0;
    public int rotation = 0;
    ElapsedTime autoTimer;

    public boolean park;

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
        this.park = false;
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

        //telemetry.addData("Battery Voltage", ctrlHub.getBatteryVoltage());
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

    public void addWaypointTaskDefault(Pose pose) {
        queue.addTask(new PointTask(new Waypoint(pose) , ptpController));
    }

    //Constructer gets in other stuff too -- power, rSlop, tSlop, passthrough
    //More detailed driving
    //Made this to not mess up my code
    public void addWaypointTask(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        //takes in the task and position
        //adds the position to the given taskList
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), ptpController));
    }

    public void addWaypointTaskOtos(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        //takes in the task and position
        //adds the position to the given taskList
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), ptpController));
    }

    public void addWaypointTaskOdo(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
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
    public void specimenPark(){
        TaskList park = new TaskList();
        addIntakeTask(park, () -> {
            outake.wristDown();
            intake.off();
            intake.initPos();
            extendoPosition = 7.3;
            liftPosition = 0;
        });

        addWaypointTask(park, new Pose(-700, -200, Math.toRadians(60)),0.9,0.1,10,false);

        queue.addTask(park);
    }

    public void specimenTelePark(){
        TaskList park = new TaskList();
        addIntakeTask(park, () -> {
            outake.readyPos();
            intake.off();
            intake.initPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        //tSlop used to be 10, rSlop used to be 0.1
        addWaypointTask(park, new Pose(-1100, 20, Math.toRadians(-90)),0.9,0.2,50,false);

        park.addTask(new DelayTask(500000));
        queue.addTask(park);
    }

    public void specimenTeleParkFaster(){
        TaskList park = new TaskList();
        addIntakeTask(park, () -> {
            outake.readyPos();
            intake.off();
            intake.initPos();
            extendoPosition = 0;
            liftPosition = 0;
        });

        //tSlop used to be 10, rSlop used to be 0.1
        addWaypointTask(park, new Pose(-1100, 0, Math.toRadians(-90)),1,0.2,50,false);

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

    public void scoringSpecimenDrive(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset){
        specimenDrive(extOffset,offsetr,offsety,offsetx);
        transferSequence2();
        scoring(scoreOffset);
    }

    public void scoringLastSpecimen(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset){
        specimen(extOffset,offsetr,offsety,offsetx);
        transferSequence();
        lastScoring(scoreOffset);
    }

    public void scoring3(){
        scoringSpecimen(0, -4, 0,0,0);
        scoringSpecimen(0.25, -6,0,0,20);
        scoringSpecimen(0.75,-7,0, 0,50);
    }

    public void scoring3Failsafe(){
        specimenFailsafe(0, -4, 0,0,0);
        specimenFailsafe(0.25, -6,0,0,20);
        specimenFailsafe(0.75,-7,0, 0,50);
    }

    public void scoringSpecimen(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset){
        specimen(extOffset,offsetr,offsety,offsetx);
        transferSequence();
        scoring(scoreOffset);
    }

    public void scoring3Different(){
        scoringSpecimenDifferent(0, -4, 0,0,0);
        scoringSpecimenDifferent(0.25, -6,0,0,20);
        scoringSpecimenDifferent(0.75,-7,0, 0,50);
    }

    public void scoringSpecimenDifferent(double extOffset,int offsetr,int offsety, int offsetx, int scoreOffset){
        specimen(extOffset,offsetr,offsety,offsetx);
        transferSequence();
        scoringDifferent(scoreOffset);
    }

    public void scoring4(){
        scoringSpecimen(0, -4, 0,0,0);
        scoringSpecimen(0.25, -6,0,0,20);
        scoringSpecimen(0.75,-7,0, -10,50);
        //scoringSpecimen(3,-11,50,-20,90);
        scoringLastSpecimen(3,-11,50,-20,90);
    }

    public void specimen(double extOffset, int offsetr, int offsety, int offsetx) {
        TaskList specimen = new TaskList();

        specimen.addTask(new CustomTask(() -> {
            autoTimer.reset();
            intake.intakeDown();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 2;
            liftPosition = 0;
            return true;
        }));

        //x = -300, y = -500, rotation = 45
        addWaypointTask(specimen, new Pose(-600+offsetx, -350+offsety, Math.toRadians(45+offsetr)),0.9,0.1,10,false);
        //addWaypointTask(specimen, new Pose(-350, -400, Math.toRadians(50)));
        /*
        addIntakeTask(specimen, () -> {
            intake.intakeDown();
            intake.in();
        });

         */
        specimen.addTask(new DelayTask(400));
        specimen.addTask(new CustomTask(() -> {
            extendoPosition = 4+extOffset;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();
            intake.setIntakeState(LOOKING);
            if(autoTimer.seconds() > 2) {
                extendoPosition = 0;
                if (autoTimer.seconds() > 3) {
                    autoTimer.reset();
                }
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE;
        }));

        //addDelayTask(specimen, 500);

        addIntakeTask(specimen, () -> {
            extendoPosition = 5;
            intake.setIntakeState(SECURED);
            intake.clawClose();
        });

        //addDelayTask(specimen,1200);

        queue.addTask(specimen);
    }

    public void specimenFailsafe(double extOffset, int offsetr, int offsety, int offsetx, int scoreOffset) {
        TaskList specimen = new TaskList();

        park = false;

        // Initial setup task
        specimen.addTask(new CustomTask(() -> {
            autoTimer.reset();
            intake.intakeDown();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 2;
            liftPosition = 0;
            return true; // Task complete
        }));

        // Move to waypoint
        addWaypointTask(specimen, new Pose(-600 + offsetx, -350 + offsety, Math.toRadians(45 + offsetr)), 0.9, 0.1, 10, false);

        specimen.addTask(new CustomTask(() -> {
            extendoPosition = 4 + extOffset;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            if (autoTimer.seconds() > 2) { // Handle timeout
                extendoPosition = 0;
                if (autoTimer.seconds() > 3) {
                    autoTimer.reset();
                }
            }

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE;
        }));

        // Intake task
        addIntakeTask(specimen, () -> {
            intake.clawClose();
            if (intake.getColor() == YELLOW) {
                queue.clear();
                specimenTelePark(); // Parking sequence
            }
        });
        /*
        // Final decision task
        specimen.addTask(new CustomTask(() -> {
            // FORCE reevaluation in case earlier detection missed updates

            telemetry.addData("Final Detected Color", intake.getColor());
            telemetry.addData("Final Park Flag", park);

            // Ensure `park` matches the most recent evaluation
            if (intake.getColor() == RED || intake.getColor() == BLUE) {
                transferSequence(); // Continue scoring4
                scoring(scoreOffset);
            }

            if (intake.getColor() == YELLOW) {
                queue.clear();
                specimenTelePark(); // Parking sequence
            }
            
            else{
                transferSequence(); // Continue scoring4
                scoring(scoreOffset);
            }
            /*
            else if (intake.getColor() == RED || intake.getColor() == BLUE) {
                transferSequence(); // Continue scoring
                scoring(scoreOffset);
            }
          //
            telemetry.update();
            return true; // Task complete
        }));

         */

        // Queue the task list
        queue.addTask(specimen);
    }

    public void specimenDrive(double extOffset, int offsetr, int offsety, int offsetx) {
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
        addWaypointTask(specimen, new Pose(-500+offsetx, -300+offsety, Math.toRadians(45+offsetr)),0.9,0.1,10,false);
        //addWaypointTask(specimen, new Pose(-350, -400, Math.toRadians(50)));
        /*
        addIntakeTask(specimen, () -> {
            intake.intakeDown();
            intake.in();
        });

         */

        specimen.addTask(new CustomTask(() -> {
            extendoPosition = 4+extOffset;
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

    public void scoring(int extraX) {
        TaskList scoring = new TaskList();

        //x = -150, y = -720, turn  = 0
        addDelayTask(scoring, 300);

        addWaypointTask(scoring, new Pose(-100+extraX, -800, Math.toRadians(0)),0.4,0.5,150,false);
        //addWaypointTask(scoring, new Pose(-150, -720, Math.toRadians(0)));

        addDelayTask(scoring, 300);

        addIntakeTask(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        //addDelayTask(scoring, 300);
        //0
        addWaypointTask(scoring, new Pose(-300, -400, 0),0.8,0.8,150,false);

        addIntakeTask(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        queue.addTask(scoring);
    }

    public void scoringDifferent(int extraX) {
        TaskList scoring = new TaskList();

        //x = -150, y = -720, turn  = 0
        addDelayTask(scoring, 300);

        addWaypointTask(scoring, new Pose(-100+extraX, -780, Math.toRadians(0)),0.4,0.5,150,false);
        //addWaypointTask(scoring, new Pose(-150, -720, Math.toRadians(0)));

        addDelayTask(scoring, 300);

        addIntakeTask(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        //addDelayTask(scoring, 300);
        //0
        addWaypointTask(scoring, new Pose(-300, -400, 0),0.8,0.8,150,false);

        addIntakeTask(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        queue.addTask(scoring);
    }

    public void scoringDrive(int extraX) {
        TaskList scoring = new TaskList();

        //x = -150, y = -720, turn  = 0
        addWaypointTask(scoring, new Pose(100+extraX, -605, Math.toRadians(0)),0.9,0.5,150,false);
        //addWaypointTask(scoring, new Pose(-150, -720, Math.toRadians(0)));

        addDelayTask(scoring, 300);

        addIntakeTask(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        addDelayTask(scoring, 300);
        //0
        addWaypointTask(scoring, new Pose(-300, -500, 0),0.8,0.8,150,false);

        addIntakeTask(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        queue.addTask(scoring);
    }

    public void lastScoring(int extraX) {
        TaskList scoring = new TaskList();

        //x = -150, y = -720, turn  = 0
        addWaypointTask(scoring, new Pose(-100+extraX, -605, Math.toRadians(0)),0.9,0.5,150,false);
        //addWaypointTask(scoring, new Pose(-150, -720, Math.toRadians(0)));

        addDelayTask(scoring, 300);

        addIntakeTask(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
            extendoPosition = 7.3;
        });

        addDelayTask(scoring, 300);
        //0
        addWaypointTask(scoring, new Pose(-300, -400, 60),0.8,0.8,150,false);

        addIntakeTask(scoring, () -> {
            outake.readyPos();
            liftPosition = 3;
        });

        queue.addTask(scoring);
    }

    public void scoringTest(int barPos, int y, int y2, int y3, double lift) {
        TaskList scoring = new TaskList();

        //addDelayTask(scoring, 500);


        //x = -150, y = -720, turn  = 0
        addWaypointTask(scoring, new Pose(0, y, Math.toRadians(0)),0.8,0.5,150,true);
        addWaypointTask(scoring, new Pose(0, y2, Math.toRadians(0)),0.8,0.5,150,false);
        //addWaypointTask(scoring, new Pose(-150, -720, Math.toRadians(0)));


        addIntakeTask(scoring, () -> {
            outake.openClaw();
            outake.wristCenter();
        });

        addDelayTask(scoring, 300);
        //0
        addWaypointTask(scoring, new Pose(barPos, y3, 0),0.8,0.5,150,false);

        addIntakeTask(scoring, () -> {
            outake.readyPos();
            liftPosition = 2;
        });

        queue.addTask(scoring);
    }

    public void transferSequence(){
        TaskList transfer = new TaskList();
        TaskList movement = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        //used to be y = -180
        addWaypointTask(movement, new Pose(0, -300, Math.toRadians(0)),0.8,0.5,150,false);

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

        addIntakeTask(transfer, () -> {
            extendoPosition = 1;
            outake.scorePosLeft();
            liftPosition = highChamberPos;

            transfering = false;
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

    public void transferSequence2(){
        TaskList transfer = new TaskList();
        TaskList movement = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);

        //used to be y = -180
        addWaypointTask(movement, new Pose(100, -120, Math.toRadians(0)),0.8,0.5,150,false);

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

        addIntakeTask(transfer, () -> {
            extendoPosition = 1;
            outake.scorePosLeft();
            liftPosition = highChamberPos;

            transfering = false;
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

    public void ttSampleTest(){
        fistSampleTT_Test();
        secondSampleTT_Test();
        thridSampleTT_Test();
    }
    public void sampleTT_Test(int x,int y,int r) {ttdriving(x, y,r);}
    public void fistSampleTT_Test() {ttdriving(-650, -700,90);}
    public void secondSampleTT_Test(){ttdriving(-950, -700,90);}
    public void thridSampleTT_Test(){ttdriving(-1240, -710,100);}


    public void ttSample(){
        fistSampleTT();
        secondSampleTT();
        //thridSampleTT();
    }

    public void ttSample3(){
        fistSampleTT();
        secondSampleTT();
        thridSampleTT();
    }

    public void ttSample3Different(){
        valuesSampleTT(-500,-720,90);
        secondSampleTT();
        thridSampleTT();
    }

    public void fistSampleTT() {ttdriving(-600, -720,90);}
    public void secondSampleTT(){ttdriving(-920, -720,90);}
    public void thridSampleTT(){ttdriving(-1230, -610,100);}

    public void valuesSampleTT(int x, int y, int rotation){
        ttdriving(x, y,rotation);
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
            intake.intakeDown();
            intake.turntableRight();
            intake.in();
            return true;
        }));
        //make these slower
        addWaypointTask(sample, new Pose(x1, y1, Math.toRadians(r1)), 0.4, 0.1, 10, false);
        //addWaypointTask(sample, new Pose(x1, y1, Math.toRadians(120)), 0.4, 0.1, 10, false);

        //addDelayTask(sample, 600);
        sample.addTask(new CustomTask(() -> {
            intake.in();
            boolean quit = false;

            if (encoderLocalizer.getPos().getR() < Math.toRadians(145)) {
                dt.drive(0.1, 0.2, -0.12);
            }
            else {
                dt.drive(0, 0, 0);
                quit = true;
            }
            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 2;
            intake.turntableLeft();
            intake.clawClose();
            return true;
        }));
        addWaypointTask(sample, new Pose(-850, -400, Math.toRadians(70)), 1, 0.1, 100, false);
        sample.addTask(new CustomTask(() -> {
            extendoPosition = 4;
            intake.out();
            intake.clawOpen();
            return intake.getColor() != YELLOW && intake.getColor() != RED && intake.getColor() != BLUE;
        }));
        queue.addTask(sample);
    }

    public void drive(int x, int y, double r){
        TaskList drive = new TaskList();

        drive.addTask(new CustomTask(() -> {
            extendoPosition = 0;
            return true;
        }));
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

        addWaypointTask(sample, new Pose(x1, y1, Math.PI), 0.8, 0.1, 200, true);
        addWaypointTask(sample, new Pose(x2, y2, Math.PI), 0.8, 0.01, 100, true);
        addWaypointTask(sample, new Pose(x3, y3, Math.PI), 0.8, 0.01, 100, true);

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

        addWaypointTask(sample, new Pose(x1, y1, Math.PI), 0.8, 0.1, 200, true);
        addWaypointTask(sample, new Pose(x2, y2, Math.PI), 0.8, 0.01, 100, false);

        sample.addTask(new CustomTask(() -> {
            extendoPosition = 3;
            return true;
        }));

        addWaypointTask(sample, new Pose(x3, y3, Math.PI), 0.8, 0.01, 100, false);

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
    public void firstSpecimen(int distance, double height) {
        TaskList scoringSpecimen = new TaskList();

        addIntakeTask(scoringSpecimen, () -> {
            liftPosition = height;
            outake.autoAutoHighRungPos();
            extendoPosition = 3;
        });
        addWaypointTask(scoringSpecimen, new Pose(-100, distance, 0),0.4,0.5,150,false);

        addDelayTask(scoringSpecimen, 200);
        addIntakeTask(scoringSpecimen, () -> {
            outake.openClaw();
            liftPosition = 2;
            extendoPosition = 3;
        });
        addDelayTask(scoringSpecimen, 300);

        addWaypointTask(scoringSpecimen, new Pose(-150, -400, Math.toRadians(50)),0.6,0.5,150,false);

        addIntakeTask(scoringSpecimen, () -> {
            outake.transferPos();
            liftPosition = 0;
            extendoPosition = 0;
            outake.readyPos();
            intake.intakeDown();
        });

        queue.addTask(scoringSpecimen);
    }

    public void firstSpecimenBitFaster(int distance, double height) {
        TaskList scoringSpecimen = new TaskList();

        addIntakeTask(scoringSpecimen, () -> {
            liftPosition = height;
            outake.autoAutoHighRungPos();
            extendoPosition = 3;
        });
        addWaypointTask(scoringSpecimen, new Pose(-100, distance, 0),0.6,0.5,150,false);

        addDelayTask(scoringSpecimen, 200);
        addIntakeTask(scoringSpecimen, () -> {
            outake.openClaw();
            liftPosition = 2;
            extendoPosition = 3;
        });
        addDelayTask(scoringSpecimen, 300);

        addWaypointTask(scoringSpecimen, new Pose(-150, -400, Math.toRadians(80)),0.7,0.5,150,false);

        addIntakeTask(scoringSpecimen, () -> {
            outake.transferPos();
            liftPosition = 0;
            extendoPosition = 0;
            outake.readyPos();
            intake.intakeDown();
        });

        queue.addTask(scoringSpecimen);
    }

    public void firstSpecimen() {
        TaskList scoringSpecimen = new TaskList();

        addIntakeTask(scoringSpecimen, () -> {
            liftPosition = 2.9;
            outake.autoAutoHighRungPos();
            extendoPosition = 3;
        });
        addWaypointTask(scoringSpecimen, new Pose(-100, -840, 0),0.9,0.5,150,false);

        addDelayTask(scoringSpecimen, 200);
        addIntakeTask(scoringSpecimen, () -> {
            outake.openClaw();
            liftPosition = 2;
            extendoPosition = 3;
        });
        addDelayTask(scoringSpecimen, 300);

        addWaypointTask(scoringSpecimen, new Pose(-150, -400, Math.toRadians(50)),0.9,0.5,150,false);

        addIntakeTask(scoringSpecimen, () -> {
            outake.transferPos();
            liftPosition = 0;
            extendoPosition = 0;
            outake.readyPos();
        });

        queue.addTask(scoringSpecimen);
    }
}