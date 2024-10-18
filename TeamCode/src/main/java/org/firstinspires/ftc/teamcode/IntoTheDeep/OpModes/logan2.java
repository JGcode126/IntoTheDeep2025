package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.CanvasOp;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.Queue.CustomTask;
import com.roboctopi.cuttlefish.Queue.DelayTask;
import com.roboctopi.cuttlefish.Queue.HybridTask;
import com.roboctopi.cuttlefish.Queue.LogTask;
import com.roboctopi.cuttlefish.Queue.PointTask;
import com.roboctopi.cuttlefish.Queue.ServoTask;
import com.roboctopi.cuttlefish.Queue.Task;
import com.roboctopi.cuttlefish.Queue.TaskList;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.utils.Line;
import com.roboctopi.cuttlefish.utils.PID;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.cuttlefishftcextensions.tasks.MotorPositionTask;
import org.firstinspires.ftc.teamcode.cuttlefishftcextensions.tasks.ServoPresetTask;
import org.firstinspires.ftc.teamcode.opmodeTypes.InitializedOpmode;
import org.firstinspires.ftc.teamcode.opmodes.util.OdoIMU;
import org.firstinspires.ftc.teamcode.tasks.AwaitServoPosTask;
import org.firstinspires.ftc.teamcode.tasks.BrakeTask;
import org.firstinspires.ftc.teamcode.tasks.FindConeTask;
import org.firstinspires.ftc.teamcode.vision.PoleLineVisV2;
import org.firstinspires.ftc.teamcode.vision.PoleLineVisV3;
import org.firstinspires.ftc.teamcode.vision.SleevePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import static com.roboctopi.cuttlefish.utils.MathUtilsKt.rFullToHalf;
import static com.roboctopi.cuttlefish.utils.MathUtilsKt.rHalfToFull;

@Autonomous(name="5x Near", group="Autoalign")
//@Disabled
public class AutoRightHighV3 extends CuttleInitOpMode {
    enum Pole
    {
        FIRST,
        NEAR_HIGH,
        FAR_HIGH,
        NEAR_MID
    }

    double initL = 0;
    double initR = 0;
    OdoIMU fusor;

    final boolean LINE_VISION_ENABLED = true;

    OpenCvWebcam grabberWebcam;
    protected OpenCvWebcam frontWebcam;
    SleevePipeline sleevePipeline;
    PoleLineVisV3 visPipeline;
    ElapsedTime autoTimer;

    public void onInit()
    {
        super.onInit();
        autoTimer = new ElapsedTime();

        setDriveUpPosition();

        initL = devs.leftEncoder .getRotation();
        initR = devs.rightEncoder.getRotation();
        fusor = new OdoIMU(imu,chassis.localizer);
        sendTelem();
        grabber.setPresetPosition(0);

        sleevePipeline = new SleevePipeline(telemetry, 320, 240);


        //Init Camera
        int cameraView = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] miniViewIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(
                cameraView,
                2, //The number of sub-containers to create
                OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY
        );



        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), miniViewIds[0]);
        grabberWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), miniViewIds[1]);



        frontWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                frontWebcam.setPipeline(sleevePipeline);
                frontWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                System.out.println("Mission failed successfully");


            }

            @Override

            public void onError(int errorCode)
            {
                System.out.println("CAMERA NOT OPENED: " + errorCode);
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        FtcDashboard.getInstance().startCameraStream(frontWebcam, 0);

        grabber.setPresetPosition(0);

        if (AutoOptions.flipMultiplier > 0) {
            telemetry.addLine("RIGHT");
        } else {
            telemetry.addLine("LEFT");
        }
        telemetry.addLine(AutoOptions.teamColor.toString());

        if (AutoOptions.hasBeenChanged == false) {
            telemetry.addLine("WARNING: Auto hasn't had options set!");

        }
        telemetry.update();
    }

    final double TILE = 609.6;

    double alignX = 0;
    double alignY = 1320;

    double highAlignX = -600;
    double highAlignY = 1300; // for the near high pole=

    double midAlignX = 130;
    double midAlignY = 1230; // for the near mid pole=
    //try 1446.7


    double searchX = alignX + 500;
    double searchY = alignY-30 ;

    int stackHeight = 5;
    Pose worldTranslation = new Pose(AutoOptions.flipMultiplier*36*25.4,-65*25.4+(searchY-1219.2),0.0);

    //TODO: Make line align on pickup more aggressive

    //TODO: Near high is over-rotating and seeing other pole
    //TODO: Far high park should just back up. Also it is on the line
    //TODO: Go for far pole more
    //TODO: Maybe decel on way to pickup
    //TODO: Initial deliver is possibly waiting for lift still
    Waypoint driveUpPosition;
    Waypoint driveUpPositionFirst; // drive up position for first cycle only, (its the same as normal driveUpPosition in normal auto but not alt auto)
    Waypoint driveUpPositionMid;

    // context: setting the drive up position has to be done after init so that the flip multiplier is set
    // so this is a thing to be overriden by alt path because it has a different drive up waypoint
    protected void setDriveUpPosition()
    {
        //The position to drive to for deliver
        driveUpPosition = new Waypoint(new Pose(AutoOptions.flipMultiplier*(alignX+40), (alignY-40), AutoOptions.flipMultiplier*(Math.PI/4+0.4)), 0.25, 0.06, 40.0, false);

        Pose near_high = new Pose(AutoOptions.flipMultiplier*POLE_DIST,0.0,0.0);
        near_high.subtract(worldTranslation,true);
        near_high.subtract(driveUpPosition.getPosition(),false);
        driveUpPosition.getPosition().setR(Math.atan2(-near_high.getX(),near_high.getY()));

        driveUpPositionFirst  = new Waypoint(new Pose(AutoOptions.flipMultiplier*(alignX+40), (alignY), AutoOptions.flipMultiplier*Math.PI/4), 0.35, 0.06, 100.0, false);
    }

    boolean park = false;

    Pole lastPole = Pole.NEAR_HIGH;
    int parkTimeout = 23500;

    PoleLineVisV3.TargetColor color = PoleLineVisV3.TargetColor.RED;

    @Override
    public void main()
    {
        super.main();

        if (AutoOptions.teamColor == AutoOptions.TeamColor.RED_TEAM)
        {
            color = PoleLineVisV3.TargetColor.RED;
        }
        else
        {
            color = PoleLineVisV3.TargetColor.BLUE;
        }

        driveUpPositionMid = new Waypoint(new Pose(AutoOptions.flipMultiplier*(midAlignX), (midAlignY), AutoOptions.flipMultiplier*(2*Math.PI*20/360+Math.PI / 2)), 0.35, 0.1, 100.0, false);
        autoTimer.reset();

        if(sleevePipeline.coneState == 0)
        {
            parkTimeout = 23500;
        }
        else if(sleevePipeline.coneState == 1)
        {
            parkTimeout = 23500;
        }
        else if(sleevePipeline.coneState == 2)
        {
            parkTimeout = 23500;

        }

        grabberWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                int w = 320;
                int h = 240;

                visPipeline = new PoleLineVisV3(w, h,chassis.localizer, color);
                grabberWebcam.setPipeline(visPipeline);
                grabberWebcam.startStreaming(w, h, OpenCvCameraRotation.UPSIDE_DOWN);
//                grabberWebcam.getExposureControl().setExposure(100, TimeUnit.MICROSECONDS);
//                grabberWebcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                grabberWebcam.getExposureControl().setMode(ExposureControl.Mode.Auto);
                FtcDashboard.getInstance().stopCameraStream();
                FtcDashboard.getInstance().startCameraStream(grabberWebcam, 0);
                visPipeline.mode = PoleLineVisV3.Mode.Pole;
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                System.out.println("CAMERA OPENING ERROR: "+errorCode);
            }
        });



        //Adjust red search
//        if(flipMultiplier <0)
//        {
//            searchY-=60;
//        }

        //Close front webcam
        try {
            frontWebcam.stopStreaming();
            frontWebcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener(){
                @Override
                public void onClose() {
                    System.out.println("FRONT CAMERA CLOSED");
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Home lift
        liftMotor.setPower(-0.6);
        chassis.queue.addTask(new CustomTask( ()->{
            if(!liftSensor.getState()&&!liftActive)
            {
                liftActive = true;
                liftMotor.positionController.onBegin();
                liftMotor.enablePositionPID(true);
                liftMotor.setPosition(liftBottom);
                armMotor.setPosition(armDeliverAngle);
                return true;
            }
            return false;
        }, false));

        chassis.queue.addTask(new CustomTask(()->{
            System.out.println("Lift homed and ready");
            return true;
        }, false));

        /*YEET START*/
        chassis.queue.addTask(new ServoTask(yeetStick, 0.55));

        chassis.queue.addTask(new MotorPositionTask(liftAboveCone,liftMotor,false,0.25f));
        //Post yeet drive
        chassis.queue.addTask(new PointTask(
                new Waypoint( new Pose(AutoOptions.flipMultiplier*alignX,alignY,0),1.0,0.3,1000,false ),
                chassis.PTPController
        ));
        chassis.queue.addTask(new ServoPresetTask(yeetStick, 0));

        TaskList yeet = new TaskList();
        yeet.addTask(new CustomTask(()->{
            return chassis.localizer.getPos().getY()>640;
        },false));
        yeet.addTask(new ServoPresetTask(yeetStick, 1));
        chassis.queue.addTask(new HybridTask(new PointTask(
                new Waypoint( new Pose(AutoOptions.flipMultiplier*alignX,alignY,0),0.4,0.3,400,false ),
                chassis.PTPController
        ),yeet));
        chassis.queue.addTask(new MotorPositionTask(liftHigh,liftMotor,false,0.25f));
//        out.addTask(new BrakeTask(300,0.5,0,chassis));
        TaskList unyeet = new TaskList();
        unyeet.addTask(new DelayTask(200));
        unyeet.addTask(new ServoPresetTask(yeetStick, 0));
        /*YEET END*/

        chassis.queue.addTask(new CustomTask(()->{
            System.out.println("Post yeet sequence");
            return true;
        }, false));

        chassis.queue.addTask(new HybridTask(cycle(Pole.FIRST),unyeet));

        chassis.queue.addTask(new CustomTask(()->{
            System.out.println("First cycle done");
            return true;
        }, false));
        //TODO: Make fix?
//        chassis.queue.addTask(new CustomTask(()->{
//            if(AutoOptions.flipMultiplier<0)
//            {
//                chassis.localizer.getPos().setY(chassis.localizer.getPos().getY()+50);
//            }
//            return true;
//        },false));

        chassis.queue.addTask(new CustomTask(()->{
            System.out.println("Check done");
            return true;
        }, false));

        doCycles();

        // final deliver the loaded cone before park
//        chassis.queue.addTask(deliver());

        chassis.queue.addTask(new CustomTask(()->{
            System.out.println("Retract");
            return true;
        }, false));

        chassis.queue.addTask(new ServoPresetTask(grabber,2));
        chassis.queue.addTask(new ServoPresetTask(wrist,1));

        // retract arm and lift
        TaskList properRetract = new TaskList();
        properRetract.addTask(new MotorPositionTask(0,armMotor,true,0.5f));
        properRetract.addTask(new MotorPositionTask(0,liftMotor,false,0.25f));

        // park, (park function returns the tasks for parking
        // it is overridden in alt path and stuff to park differently
        TaskList park = parkMove();
        chassis.queue.addTask(new HybridTask(park,properRetract));
    }
    public void doCycles()
    {
        chassis.queue.addTask(new LogTask("Starting cycle 1"));
        chassis.queue.addTask( cycle(Pole.NEAR_HIGH) );
        chassis.queue.addTask(new LogTask("Starting cycle 2"));
        chassis.queue.addTask( cycle(Pole.NEAR_HIGH) );
        chassis.queue.addTask(new LogTask("Starting cycle 3"));
        chassis.queue.addTask( cycle(Pole.NEAR_HIGH) );
        chassis.queue.addTask(new LogTask("Starting cycle 4"));
        chassis.queue.addTask( cycle(Pole.NEAR_HIGH) );
        chassis.queue.addTask(new LogTask("Starting cycle 5"));
        chassis.queue.addTask(cycle(Pole.NEAR_HIGH));
        chassis.queue.addTask(new LogTask("Cycles Complete"));
    }


    double visionStart = 0;

    double pastBump = 0.0;

    public TaskList alignToPole(Pole target)
    {

        TaskList miniQueue = new TaskList();
        //Take snapshot using pole vision
        miniQueue.addTask(new CustomTask(()->{
            visionStart = System.nanoTime();
            visPipeline.pole_frames = 0;
            return true;
        },false));
        miniQueue.addTask(new CustomTask(()->{
            visPipeline.mode = PoleLineVisV3.Mode.Pole;
            if(visPipeline.pole_frames >0)
            {
//                visPipeline.mode = PoleLineVisV2.Mode.Off;
                visPipeline.pole_frames = 0;
                System.out.println("VISTIME: " + (float)(System.nanoTime()-visionStart)/(1000.0*1000.0));
                return true;
            }
            return false;
        },false));

        //Save pole vision data
        miniQueue.addTask(new CustomTask(()->{
            double target_angle = AutoOptions.flipMultiplier * Math.PI / 4;
            double angle = visPipeline.seenR;

            double horiz_fov = 64.5f / 180.0f * Math.PI;

            double coeff = 0.77;

            if(target == Pole.NEAR_HIGH)
            {
                coeff = 0.725;
            }
            if(target == Pole.NEAR_MID)
            {
                coeff = 0.6;
            }

            double bump_angle = coeff*visPipeline.polevec_x * horiz_fov / 2;

            final double MAX_DEV = 7 / 180.0 * Math.PI;


            visPipeline.polevec_save_x = visPipeline.polevec_x;
            visPipeline.polevec_save_y = visPipeline.polevec_y;

            angle -= bump_angle;

//            if ((angle - target_angle) > MAX_DEV)
//            {
//                angle = target_angle + MAX_DEV;
//            }
//            else if ((angle - target_angle) < -MAX_DEV)
//            {
//                angle = target_angle - MAX_DEV;
//            }

            if(Math.abs(bump_angle)<Math.PI/12)
            {
                pastBump = bump_angle;
            }

            double x = Math.sin(-angle);
            double y = Math.cos(-angle);
            updateLine(target,x,y);

            //TODO: Double seenX?
            //Adjust next angle
//            poleLine.setPx(visPipeline.seenX);
//            poleLine.setPy(visPipeline.seenY);
//            poleLine.setVx(x);
//            poleLine.setVy(y);

//            if(target == Pole.NEAR_MID)
//            {
////                driveUpPositionMid.getPosition().setR(angle);
//            }

            return true;
        }, false));
        return  miniQueue;
    }

    static double POLE_DIST = 25.4*24;
    Line visionLine = new Line(0.0,0.0,0.0,0.0);
    public void updateLine(Pole target,double x, double y)
    {
        poleLine.setPx(visPipeline.seenX);
        poleLine.setPy(visPipeline.seenY);
        poleLine.setVx(x);
        poleLine.setVy(y);

        visionLine.setPx(visPipeline.seenX);
        visionLine.setPy(visPipeline.seenY);
        visionLine.setVx(x);
        visionLine.setVy(y);

        if(target == Pole.NEAR_HIGH || target == Pole.FIRST)
        {
            Pose near_high = new Pose(AutoOptions.flipMultiplier*POLE_DIST,0.0,0.0);
            near_high.subtract(worldTranslation,true);

            ArrayList<Pose> wrong_poles = new ArrayList<Pose>();
            wrong_poles.add(new Pose(AutoOptions.flipMultiplier*POLE_DIST,POLE_DIST,0.0));

            double correctDist = Math.abs(poleLine.getDist(near_high));
            System.out.println("correctDist: "+correctDist);
            boolean pole_is_correct = true;
            for(int i = 0; i < wrong_poles.size(); i++)
            {
                wrong_poles.get(i).subtract(worldTranslation,true);
                double wrong_dist = Math.abs(poleLine.getDist(wrong_poles.get(i)));
                System.out.println("wrong_dist: "+wrong_dist);
                if(wrong_dist<correctDist)
                {
                    pole_is_correct = false;
                    break;
                }
            }

            if(!pole_is_correct)
            {
                poleLine.setVx(near_high.getX()-visPipeline.seenX);
                poleLine.setVy(near_high.getY()-visPipeline.seenY);
                poleLine.normalize();
                System.out.println("WRONG VISION POLE!!");
            }
        }
    }

    Line poleLine = new Line(0.0,0.0,0.0,0.0);

    public TaskList visionDrop(Pole target)
    {

        TaskList miniQueue = new TaskList();//Drive into pole until drop is triggered
        miniQueue.addTask(alignToPole(target));
        miniQueue.addTask(new ServoPresetTask(wrist,2));
        TaskList alignToPole = new TaskList();
        alignToPole.addTask(alignToPole(target));
        if(target == Pole.FAR_HIGH)
        {
            alignToPole.addTask(alignToPole(target));
        }

        miniQueue.addTask(new HybridTask(
                new FindConeTask(chassis,frontBeamBreak,poleLine,0.35,0.0,500,true,500),
                alignToPole
        ));
        miniQueue.addTask(new CustomTask(()->{
            visPipeline.mode = PoleLineVisV3.Mode.Off;
            return true;
        },false));
        //Drop the cone
        miniQueue.addTask(new ServoPresetTask(grabber,2));
        miniQueue.addTask(new ServoTask(wrist, wrist.getPreset(2)+0.15));
        miniQueue.addTask(new DelayTask(200));
        return miniQueue;
    }




    protected TaskList parkMove()
    {
        TaskList parkQueue = new TaskList();

        //Go to the center of the park area
        parkQueue.addTask(new CustomTask(()->{
            if(lastPole == Pole.NEAR_HIGH)
            {
                parkQueue.addTask(new PointTask(
                        new Waypoint( new Pose(AutoOptions.flipMultiplier*alignX,alignY-65,AutoOptions.flipMultiplier*Math.PI/2),0.6,1.0,70.0,false ),
                        chassis.PTPController
                ));
            }

            //Shift to the side based on vision
            if(sleevePipeline.coneState == 0)
            {
                parkQueue.addTask(new PointTask(
                        new Waypoint( new Pose(-TILE,searchY,AutoOptions.flipMultiplier*Math.PI/2),0.5,0,0,false ),
                        chassis.PTPController
                ));
            }
            else if(sleevePipeline.coneState == 1)
            {
                parkQueue.addTask(new PointTask(
                        new Waypoint( new Pose(AutoOptions.flipMultiplier*alignX,searchY,AutoOptions.flipMultiplier*Math.PI/2),0.5,0.0,0,false ),
                        chassis.PTPController
                ));
            }
            else if (sleevePipeline.coneState == 2)
            {
                parkQueue.addTask(new PointTask(
                        new Waypoint( new Pose(TILE,searchY,AutoOptions.flipMultiplier*Math.PI/2),0.5,0,0,false ),
                        chassis.PTPController
                ));
            }

            return true;
        },false));
        return parkQueue;
    }


    /**Sequence of yeet*/
    /*public TaskList yeetSequence()
    {
        //Raise the lift a to avoid crash
        TaskList out = new TaskList();

        out.addTask(new MotorPositionTask(liftAboveCone,liftMotor,false,0.25f));

        //Preset yeet stick out
        if (flipMultiplier < 0) {
//            out.addTask(new ServoPresetTask(yeetStick, 2));
        }

        //Pre yeet drive
        if (flipMultiplier < 0)
        {
            out.addTask(new PointTask(
                    new Waypoint( new Pose(flipMultiplier*alignX, TILE + 470,0),1.0,0.3,700+60,false ),
                    chassis.PTPController
            ));
        }
        else
        {
            out.addTask(new PointTask(
                    new Waypoint( new Pose(flipMultiplier*alignX, TILE + 380,0),1.0,0.3,700+60,false ),
                    chassis.PTPController
            ));
        }
//
//        //YEET
//        if (flipMultiplier < 0) {
//            out.addTask(new ServoPresetTask(yeetStick, 1));
//        }
//        else
//        {
//            out.addTask(new ServoPresetTask(yeetStick, 1));
//        }

        //Post yeet drive
        out.addTask(new PointTask(
                new Waypoint( new Pose(flipMultiplier*alignX,alignY,0),1.0,0.3,700,false ),
                chassis.PTPController
        ));

        out.addTask(new PointTask(
                new Waypoint( new Pose(flipMultiplier*alignX,alignY,0),0.6,0.3,500,false ),
                chassis.PTPController
        ));

        //Unyeet the stick
        out.addTask(new ServoPresetTask(yeetStick, 0));

        //Raise the lift to deliver
        out.addTask(new MotorPositionTask(liftHigh,liftMotor,false,0.25f));

        //Final drive after yeet retract
        out.addTask(new PointTask(
                new Waypoint( new Pose(flipMultiplier*alignX,alignY,0),0.35,0.15,200,false ),
                chassis.PTPController
        ));
        out.addTask(new ServoPresetTask(yeetStick, 1));
        return out;
    }*/
    public TaskList yeetSequence()
    {
        //Raise the lift a to avoid crash
        TaskList out = new TaskList();
        out.addTask(new ServoTask(yeetStick, 0.55));

        out.addTask(new MotorPositionTask(liftAboveCone,liftMotor,false,0.25f));
        //Post yeet drive
        out.addTask(new PointTask(
                new Waypoint( new Pose(AutoOptions.flipMultiplier*alignX,alignY,0),1.0,0.3,1000,false ),
                chassis.PTPController
        ));
        out.addTask(new ServoPresetTask(yeetStick, 0));

        out.addTask(new PointTask(
                new Waypoint( new Pose(AutoOptions.flipMultiplier*alignX,alignY,0),0.4,0.3,400,false ),
                chassis.PTPController
        ));
        out.addTask(new MotorPositionTask(liftHigh,liftMotor,false,0.25f));
        out.addTask(new ServoPresetTask(yeetStick, 1));
//        out.addTask(new BrakeTask(300,0.5,0,chassis));
        out.addTask(new DelayTask(200));
        out.addTask(new ServoPresetTask(yeetStick, 0));
        //Raise the lift to deliver

        //Final drive after yeet retract
//        out.addTask(new PointTask(
//                new Waypoint( new Pose(flipMultiplier*alignX,alignY,0),0.35,0.15,200,false ),
//                chassis.PTPController
//        ));
        return out;
    }

    PID drivebackPID = new PID(1.0/(100.0),0.0,1.5/(100.0),0.0,0.0);
    public TaskList cycle(Pole deliverTo)
    {
        TaskList out = new TaskList();
        out.addTask(new CustomTask(()->{
            if(park)
            {
                out.getQueue().clear();
            }
            else
            {
                lastPole = deliverTo;
            }
            return true;
        },false));




        //Get the arm ready to deliver
        TaskList armArm = new TaskList();
        armArm.addTask(new DelayTask(100));
        armArm.addTask(new ServoPresetTask(yeetStick, 0));
        if(deliverTo == Pole.NEAR_MID)
        {
            armArm.addTask(new MotorPositionTask(liftMid+10, liftMotor, false, 0.5f));
        }
        else
        {
            armArm.addTask(new MotorPositionTask(liftHigh, liftMotor, false, 0.5f));
        }
        armArm.addTask(new MotorPositionTask(armDeliverAngle, armMotor, true, (float)armDeliverAngle));
        if(deliverTo == Pole.FAR_HIGH)
        {
            armArm.addTask(new ServoTask(wrist, wrist.getPreset(2)+0.15));
        }
        else
        {
            armArm.addTask(new ServoTask(wrist, wrist.getPreset(2)+0.05));
        }

        armArm.addTask(new MotorPositionTask(armDeliverAngle, armMotor, true, 0.4f));

        //Drive to pole
        if(deliverTo == Pole.FIRST)
        {
            out.addTask(new ServoPresetTask(wrist,2));
            out.addTask(new PointTask(
                    driveUpPositionFirst,
                    chassis.PTPController
            ));
            out.addTask(new PointTask(
                    driveUpPositionFirst,
                    chassis.PTPController
            ));
        }
        else if(deliverTo == Pole.NEAR_HIGH)
        {
            TaskList driveup = new TaskList();

            driveup.addTask(new PointTask(
                    new Waypoint(new Pose(AutoOptions.flipMultiplier*(alignX + 200), alignY, AutoOptions.flipMultiplier*Math.PI / 2), 0.6, 1, 80.0, true),
                    chassis.PTPController
            ));

            driveup.addTask(new PointTask(
                    driveUpPosition,
                    chassis.PTPController
            ));
            driveup.addTask(new BrakeTask(50,0.4,0.1,chassis));
            driveup.addTask(new PointTask(
                    driveUpPosition,
                    chassis.PTPController
            ));
            out.addTask(new HybridTask(armArm,driveup));
        }
        else if(deliverTo == Pole.NEAR_MID)
        {
            TaskList driveup = new TaskList();


            driveup.addTask(new PointTask(
                    new Waypoint(new Pose(AutoOptions.flipMultiplier*(alignX + 200), midAlignY, AutoOptions.flipMultiplier*Math.PI / 2), 0.6, 1, 80.0, false),
                    chassis.PTPController
            ));

            driveup.addTask(new PointTask(
                    driveUpPositionMid,
                    chassis.PTPController
            ));
            out.addTask(new HybridTask(armArm,driveup));

        }
        else if(deliverTo == Pole.FAR_HIGH)
        {
            TaskList driveup = new TaskList();
            driveup.addTask(new PointTask(
                    new Waypoint(new Pose(AutoOptions.flipMultiplier*(highAlignX), highAlignY+40, AutoOptions.flipMultiplier*Math.PI / 2), 1.0, 1, 760, false),
                    chassis.PTPController
            ));
//            driveup.addTask(new BrakeTask(100,0.4,0.1,chassis));
//            driveup.addTask(new PointTask(
//                    new Waypoint(new Pose(flipMultiplier*(highAlignX +40), (highAlignY +40), flipMultiplier*Math.PI / 4 * 3), 0.3, 0.03, 20.0, false),
//                    chassis.PTPController
//            ));
            driveup.addTask(new PointTask(
                    new Waypoint(new Pose(AutoOptions.flipMultiplier*-400, alignY, AutoOptions.flipMultiplier*2.05), 0.5, 0.06, 90.0, false),
                    chassis.PTPController
            ));
            out.addTask(new HybridTask(armArm,driveup));
//            out.addTask(new CustomTask(()->{
////                System.out.println("Position: " + chassis.localizer.getPos());
//                return false;
//            },false));
        }


        out.addTask(visionDrop(deliverTo));

        //Back Away
        TaskList goBack = new TaskList();

        if(deliverTo == Pole.NEAR_HIGH||deliverTo == Pole.FIRST)
        {
            goBack.addTask(new PointTask(
                    new Waypoint(new Pose(AutoOptions.flipMultiplier * (alignX - 60), (alignY + 60), AutoOptions.flipMultiplier * Math.PI / 4), 0.6, 0.4, 120.0, true)
                    , chassis.PTPController));
        }
        else if(deliverTo == Pole.NEAR_MID)
        {
            goBack.addTask(new PointTask(
                    new Waypoint(new Pose(AutoOptions.flipMultiplier * (midAlignX-50), (midAlignY + 35), driveUpPositionMid.getPosition().getR()), 0.6, 0.4, 120.0, true)
                    , chassis.PTPController));
        }
        else if(deliverTo == Pole.FAR_HIGH)
        {
//            goBack.addTask(new PointTask(
//                new Waypoint(new Pose(flipMultiplier * (highAlignX - 20), (highAlignY - 20), flipMultiplier * Math.PI / 4 * 3), 0.6, 0.4, 120.0, true)
//            , chassis.PTPController));

            goBack.addTask(new PointTask(
                    new Waypoint(new Pose(AutoOptions.flipMultiplier*-460, alignY+30, AutoOptions.flipMultiplier*2.0), 0.8, 1, 140, false),
                    chassis.PTPController
            ));
        }



        //Drive Back
        out.addTask(new CustomTask(()->{
            if(autoTimer.milliseconds() > parkTimeout || stackHeight == 0)
            {
                park = true;
                for(int i = 0; i < 10; i++)
                {
                    System.out.println("OUT OF TIME: "+ autoTimer.milliseconds());
                }
                out.addTask(goBack);
            }
            else
            {
                if(deliverTo == Pole.NEAR_HIGH||deliverTo == Pole.FIRST)
                {
                    goBack.addTask(new PointTask(
                            new Waypoint(new Pose(AutoOptions.flipMultiplier * (searchX+80), searchY, AutoOptions.flipMultiplier * Math.PI / 2), 0.7, 0.1, 150.0, false),
                            chassis.PTPController
                    ));
                }
                else if(deliverTo == Pole.NEAR_MID)
                {
                    goBack.addTask(new PointTask(
                            new Waypoint(new Pose(AutoOptions.flipMultiplier * (searchX+70), searchY, AutoOptions.flipMultiplier * Math.PI / 2), 0.7, 0.1, 150.0, false),
                            chassis.PTPController
                    ));
                }
                else if(deliverTo == Pole.FAR_HIGH)
                {
//                    goBack.addTask(new PointTask(
//                            new Waypoint(new Pose(AutoOptions.flipMultiplier * (searchX-30), searchY, AutoOptions.flipMultiplier * Math.PI / 2), 1.0, 0.1, 150.0, false),
//                            chassis.PTPController
//                    ));
                    goBack.addTask(new CustomTask(()->{
                        chassis.PTPController.followLine(new Line(0,searchY,-AutoOptions.flipMultiplier*1.0,0.0),-1.0,drivebackPID);
                        double speed = chassis.localizer.getSpeed();
                        double stopDist = 65.3242*speed*speed+96.178*speed;
                        if(chassis.localizer.getPos().getX()*AutoOptions.flipMultiplier >520-stopDist)
                        {
                            System.out.println("stopDist: "+stopDist);
                            chassis.mecanumController.setVec(new Pose(0.0,0.0,0.0),false,0.0,0.0);
                            return true;
                        }
                        return false;
                    },false));
                }
                goBack.addTask(new ServoPresetTask(grabber, 1));
                goBack.addTask(new BrakeTask(100,0.4,0.1,chassis));

                TaskList retractArm = new TaskList();
                retractArm.addTask(new MotorPositionTask(-armPickupAngle, armMotor, true, (float)armPickupAngle));
                retractArm.addTask(new MotorPositionTask(liftBottom + 60 + coneHeight * stackHeight, liftMotor, false, 0.25f));
                retractArm.addTask(new ServoPresetTask(wrist, 1));
                retractArm.addTask(new MotorPositionTask(-armPickupAngle, armMotor, true, (float)(armPickupAngle)/2));
                retractArm.addTask(new ServoPresetTask(grabber, 1));
                retractArm.addTask(new CustomTask(()->{
                    if(deliverTo == Pole.NEAR_MID)
                    {
                        if(AutoOptions.flipMultiplier*chassis.localizer.getPos().getX()>230)
                        {
                            retractArm.addTask(alignToLine());
                            return true;
                        }
                    }
                    else
                    {
                        if(AutoOptions.flipMultiplier*chassis.localizer.getPos().getX()>200)
                        {
                            retractArm.addTask(alignToLine());
                            return true;
                        }
                    }
                    return false;
                },false));
                out.addTask(new HybridTask(goBack,retractArm));

                out.addTask(pickup());
            }
            return true;
        },false));



        return out;
    }

    public TaskList pickup()
    {
        TaskList pickupCone = new TaskList();

        TaskList stop = new TaskList();
        //Pickup a cone

//        if (LINE_VISION_ENABLED) {
//            pickupCone.addTask(new HybridTask(alignToLine(),stop));
//        }
//        else
//        {
//            pickupCone.addTask(stop);
//        }
//        pickupCone.addTask(stop);
        //Lower stack height because its a new cycle
        stackHeight--;

        pickupCone.addTask(new FindConeTask(chassis, backBeamBreak, new Line(AutoOptions.flipMultiplier*searchX, searchY, -AutoOptions.flipMultiplier*1, 0), -0.175,false));
        pickupCone.addTask(new BrakeTask(300,0.4,0.1,chassis));
        pickupCone.addTask(new MotorPositionTask(liftBottom + coneHeight * stackHeight, liftMotor, false, 0.25f));
        pickupCone.addTask(new ServoPresetTask(grabber, 0));
        pickupCone.addTask(new AwaitServoPosTask(grabberEncoder,1670,true));
        pickupCone.addTask(new MotorPositionTask(liftAboveCone + coneHeight * stackHeight+150, liftMotor, true, (185f)*(float)liftMotor.positionController.getScale()));

        return pickupCone;
    }

    public TaskList alignToLine()
    {
        TaskList align = new TaskList();
        align.addTask(new CustomTask(() -> {
            visPipeline.mode = PoleLineVisV3.Mode.Line;
            visPipeline.line_frames = 0;
            return true;
        }, false));
        align.addTask(new CustomTask(() -> {
//            if (visPipeline.mode != PoleLineVisV1.Mode.Line) {
//            }

            return visPipeline.line_frames > 0;
        }, false));

        align.addTask(new CustomTask(() -> {

            double h = liftMotor.positionController.getPosition();//liftBottom + 60 + coneHeight * stackHeight;
            double xTotal = 2.0 * visPipeline.averageY / 240.0 - 1.0;
            double yTotal = (2.0 * visPipeline.averageX / 320.0 - 1.0) * (320.0 / 240);
            double scalar = (Math.tan(Math.PI * 46.6 / 360.0) * (138.0 + h));
            double xCenter = xTotal * scalar;
            double yCenter = yTotal * scalar;

            double rTweak = -rFullToHalf(rHalfToFull(visPipeline.angle) - Math.PI / 2) + rFullToHalf(visPipeline.seenR - Math.PI / 2);

            double armLength = Math.abs(Math.sin(armMotor.positionController.getPosition()))*240;
            double rXOffset = AutoOptions.flipMultiplier*Math.sin(visPipeline.seenR-Math.PI/2)*armLength;

            System.out.println("RTWEAK: " + rTweak);
            visPipeline.saveAverageX = visPipeline.averageX;
            visPipeline.saveAverageY = visPipeline.averageY;
            visPipeline.saveAngle = visPipeline.angle;

            rTweak = 0;
            if (Math.abs(rTweak) < 0.5) {
                double currentY = chassis.localizer.getPos().getY();
                double coeff = 1.0;
//                double correction = coeff*(-(flipMultiplier*yCenter )-(visPipeline.seenY - searchY)+rXOffset);
                double correction = -((AutoOptions.flipMultiplier * yCenter + visPipeline.seenY + rXOffset) - searchY);
                if(correction > 0)
                {
//                    correction = 0;
                }
                System.out.println("CORRECTING: " + correction);
                System.out.println("yCenter: " + (yCenter ));
                System.out.println("seen: " + visPipeline.seenY);
                System.out.println("rXOffset: " + rXOffset);

                chassis.localizer.getPos().add(new Pose(0,correction,rTweak),false);
                chassis.localizer.getPPos().add(new Pose(0,correction,rTweak),false);

//                chassis.localizer.getPos().add(new Pose(0, correction, rTweak), false);
//                chassis.localizer.getPPos().add(new Pose(0, correction, rTweak), false);
            }
            return true;
        }, false));


        align.addTask(new CustomTask(() -> {
            visPipeline.mode = PoleLineVisV3.Mode.Off;
            visPipeline.line_frames = 0;
            return true;
        }, false));
        return align;
    }


    public void onEnd()
    {
//        try {
//            webcam.stopStreaming();
//            webcam.closeCameraDevice();
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
    }

    double goalAngle = 0.0;
    public void mainLoop()
    {
        super.mainLoop();
        fusor.loop();
        sendTelem();
        System.out.println("dT: "+(double)dT/(1000.0*1000.0));
        System.out.println("Position: "+chassis.localizer.getPos());


    }
    protected void sendTelem()
    {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("L",chassis.devs.leftEncoder.getRotation());
        packet.put("R",chassis.devs.rightEncoder.getRotation());
        packet.put("S",chassis.devs.sideEncoder.getRotation());
//        double xPos = chassis.localizer.getPos().getX()/25.4+36;
//        double yPos = chassis.localizer.getPos().getY()/25.4-65+(searchY-1219.2)/24.5;
//        double rPos = chassis.localizer.getPos().getR();

        Pose pos = tranformToScreen(chassis.localizer.getPos());


        packet.fieldOverlay().setFill("#FF0000").fillPolygon(
                new double[]{pos.getX()+rotateX(6,7,pos.getR()),pos.getX()+rotateX(6,-7,pos.getR()),pos.getX()+rotateX(-6,-7,pos.getR()),pos.getX()+rotateX(-6,7,pos.getR())},
                new double[]{pos.getY()+rotateY(6,7,pos.getR()),pos.getY()+rotateY(6,-7,pos.getR()),pos.getY()+rotateY(-6,-7,pos.getR()),pos.getY()+rotateY(-6,7,pos.getR())}
        );

        double lineLength = 500;
        Pose lineP1 = tranformToScreen(poleLine.getPx(),poleLine.getPy(),0);
        Pose lineP2 = tranformToScreen(poleLine.getPx()+poleLine.getVx()*lineLength,poleLine.getPy()+poleLine.getVy()*lineLength,0);
        packet.fieldOverlay().setStroke("#00FF00").strokeLine(
                lineP1.getX(),lineP1.getY(),
                lineP2.getX(),lineP2.getY()
        );

        Pose vLineP1 = tranformToScreen(visionLine.getPx(),visionLine.getPy(),0);
        Pose vLineP2 = tranformToScreen(visionLine.getPx()+visionLine.getVx()*lineLength,visionLine.getPy()+visionLine.getVy()*lineLength,0);
        packet.fieldOverlay().setStroke("#0000FF").strokeLine(
                vLineP1.getX(),vLineP1.getY(),
                vLineP2.getX(),vLineP2.getY()
        );
        packet.fieldOverlay().setStroke("#FFFF00").strokeLine(
                pos.getX(),pos.getY(),
                pos.getX()-Math.sin(driveUpPosition.getPosition().getR())*48,pos.getY()+Math.cos(driveUpPosition.getPosition().getR())*48
        );
        System.out.println("driveUpAngle: "+driveUpPosition.getPosition().getR());


        Pose near_high = new Pose(AutoOptions.flipMultiplier*POLE_DIST,0.0,0.0);
        near_high.subtract(worldTranslation,true);
        Pose highScreen = tranformToScreen(near_high);
        packet.fieldOverlay().setFill("#FF00FF").fillRect(
                highScreen.getX()-1,highScreen.getY()-1,
                2,2
        );


        ArrayList<Pose> wrong_poles = new ArrayList<Pose>();
        wrong_poles.add(new Pose(AutoOptions.flipMultiplier*POLE_DIST,POLE_DIST,0.0));
        for(int i = 0; i < wrong_poles.size(); i++)
        {
            wrong_poles.get(i).subtract(worldTranslation,true);
            Pose screenWrong = tranformToScreen(wrong_poles.get(i));

            packet.fieldOverlay().setFill("#FF00FF").fillRect(
                    screenWrong.getX()-1,screenWrong.getY()-1,
                    2,2
            );


        }


        dashboard.sendTelemetryPacket(packet);
    }

    private Pose tranformToScreen(Pose position)
    {
        Pose inter = position.clone();
        inter.add(worldTranslation,true);
        return new Pose(inter.getX()/25.4,inter.getY()/25.4,inter.getR());
    }
    private Pose tranformToScreen(double x, double y, double r)
    {
        Pose inter = new Pose(x,y,r);
        inter.add(worldTranslation,true);
        return new Pose(inter.getX()/25.4,inter.getY()/25.4,inter.getR());
    }

    private double rotateX(double x, double y, double r)
    {
        return Math.cos(r)*x-Math.sin(r)*y;
    }
    private double rotateY(double x, double y, double r)
    {
        return Math.cos(r)*y+Math.sin(r)*x;
    }
}

