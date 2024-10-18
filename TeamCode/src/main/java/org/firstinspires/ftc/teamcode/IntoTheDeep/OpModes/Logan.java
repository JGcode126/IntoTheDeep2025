package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.AdvancedMecanumController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.localizer.TwoEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.BrakeTask;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.FollowLineTask;
import com.roboctopi.cuttlefish.queue.ForkTask;
import com.roboctopi.cuttlefish.queue.MotorPowerTask;
import com.roboctopi.cuttlefish.queue.ServoTask;
import com.roboctopi.cuttlefish.queue.SmoothPointTask;
import com.roboctopi.cuttlefish.queue.Task;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TimeoutTask;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.tasks.MotorPositionTask;
import com.roboctopi.cuttlefishftcbridge.tasks.ServoPresetTask;
import com.roboctopi.cuttlefishftcbridge.utils.CurrentAlliance;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.InitializedOpmode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.competition.FTCDashConfig;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.PropPipelineV1;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;


public class ApriltagFarCycleAuto extends CuttleInitOpMode {
    AdvancedMecanumController aCtrl;
    CurrentAlliance.AllianceTypes alliance = CurrentAlliance.AllianceTypes.BLUE;
    double isBlue = 1;
    double isRed = 0;
    double flipMultiplier = 1.0;

    /* --- Cameras --- */
    OpenCvWebcam propWebcam;
    OpenCvWebcam apriltagWebcam;
    PropPipelineV1 propPipeline;
    AprilTagDetectionPipeline apriltagPipeline;
    double fx = 893.915;
    double fy = 893.915;
    double cx = 704.626;
    double cy = 361.659;


    // UNITS ARE METERS
    double tagsize = 50.0/1000.0;


    double oldForceLimit;

    double boardDist = 0.0;
    int cycleNumber = 0;

    boolean leftFilled = false;
    boolean rightFilled = false;

    boolean atBoard = false;
    BarfState secondCycleState = BarfState.BOARD;

    long startTime = 0;

    int compileCycle = 0;
    TaskList transferQueue = new TaskList();

    int failedTransfers = 0;

    int autoDelay = 8000;
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public void onInit()
    {
        super.onInit();
        purplePixelPlacer.setPosition(PPP_OUT);
        if(alliance == CurrentAlliance.AllianceTypes.RED)
        {
            DROPDOWN_ROLLER_HALF_EXTEND = 0.37;
        }

        aCtrl = new AdvancedMecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor,encoderLocalizer);
        oldForceLimit = aCtrl.getMax_force();
        int cameraView = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] miniViewIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(
                cameraView,
                2, //The number of sub-containers to create
                OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY
        );

        propWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"),miniViewIds[0]);
        apriltagWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), miniViewIds[1]);

        System.out.println("auto aliance: "+alliance);
        apriltagPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy,encoderLocalizer,imu_fusion,alliance,-1232);
        propPipeline = new PropPipelineV1(telemetry,320,240,-20+(int)isRed*40);

        propWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                if(propWebcam.getFocusControl().isFocusLengthSupported())
                {
                    propWebcam.getFocusControl().setMode(FocusControl.Mode.Fixed);
                    propWebcam.getFocusControl().setFocusLength(1023);
                    propWebcam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.AUTO);
                    System.out.println("FOCUS SUPPORTED; Focus length: "+propWebcam.getFocusControl().getFocusLength()+", minLength: "+propWebcam.getFocusControl().getMinFocusLength()+", maxLength: "+propWebcam.getFocusControl().getMaxFocusLength());

                }
                else
                {
                    System.out.println("FOCUS NOT SUPPORTED");
                }
                if(propWebcam.getExposureControl().isExposureSupported())
                {
                    System.out.println("Exposure IS SUPPORTED "+  propWebcam.getExposureControl().getMinExposure(TimeUnit.MICROSECONDS));
                    propWebcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                    //No less than 5ms
                    propWebcam.getExposureControl().setExposure(100, TimeUnit.MILLISECONDS);
                    propWebcam.getGainControl().setGain(10);
                }
                else
                {
                    System.out.println("Exposure NOT SUPPORTED");
                }
                propWebcam.setPipeline(propPipeline);
                if(alliance == CurrentAlliance.AllianceTypes.BLUE)
                {
                    propPipeline.alliance = 0;
                }
                else
                {
                    propPipeline.alliance = 1;
                }
                propWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                System.out.println("Mission succeeded successfully");
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
        dash.startCameraStream(propWebcam, 0);

        boolean lastLBumper = false;
        boolean lastRBumper = false;
        boolean lastA = false;
        while(opModeInInit())
        {
            if(gamepad1.left_bumper && !lastLBumper)
            {
                lastLBumper = true;
                autoDelay -= 1000;
            }
            else if(!gamepad1.left_bumper)
            {
                lastLBumper = false;
            }

            if(gamepad1.right_bumper && !lastRBumper)
            {
                lastRBumper = true;
                autoDelay += 1000;
            }
            else if(!gamepad1.right_bumper)
            {
                lastRBumper = false;
            }

            if(gamepad1.a && !lastA)
            {
                lastA = true;
                if(secondCycleState == BarfState.BOARD)
                {
                    secondCycleState = BarfState.YEET;
                }
                else if (secondCycleState == BarfState.YEET)
                {
                    secondCycleState = BarfState.NONE;
                }
                else
                {
                    secondCycleState = BarfState.BOARD;
                }
            }
            else if(!gamepad1.a)
            {
                lastA = false;
            }

            if(propPipeline.started)
            {
                telemetry.addData("Position",propPipeline.propPosition);
            }
            else
            {
                telemetry.addData("Position","INITIALIZING... PLEASE WAIT");
            }
            telemetry.addData("Delay",autoDelay/1000);
            telemetry.addData("Second Cycle:",secondCycleState);
            telemetry.update();
        }
    }
    public void openApriltag()
    {
        apriltagWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                if(apriltagWebcam.getFocusControl().isFocusLengthSupported())
                {
                    apriltagWebcam.getFocusControl().setMode(FocusControl.Mode.Fixed);
                    apriltagWebcam.getFocusControl().setFocusLength(1023);
                    apriltagWebcam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.AUTO);
                    System.out.println("FOCUS SUPPORTED; Focus length: "+apriltagWebcam.getFocusControl().getFocusLength()+", minLength: "+apriltagWebcam.getFocusControl().getMinFocusLength()+", maxLength: "+apriltagWebcam.getFocusControl().getMaxFocusLength());

                }
                else
                {
                    System.out.println("FOCUS NOT SUPPORTED");
                }
                if(apriltagWebcam.getExposureControl().isExposureSupported())
                {
                    System.out.println("Exposure IS SUPPORTED "+  apriltagWebcam.getExposureControl().getMinExposure(TimeUnit.MICROSECONDS));
                    apriltagWebcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                    //No less than 5ms
                    apriltagWebcam.getExposureControl().setExposure(4, TimeUnit.MILLISECONDS);

                }
                else
                {
                    System.out.println("Exposure NOT SUPPORTED");
                }
                apriltagWebcam.setPipeline(apriltagPipeline);
                apriltagWebcam.startStreaming(1280,800, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().stopCameraStream();
                FtcDashboard.getInstance().startCameraStream(apriltagWebcam, 0);

                System.out.println("Apriltag Mission succeeded successfully");
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
    }
    int position;
    public void main()
    {
        super.main();
        elapsedTime.reset();
        startTime = System.currentTimeMillis();
        homeLift();

        try {
            propWebcam.stopStreaming();
            propWebcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener(){
                @Override
                public void onClose() {
                    System.out.println("SIDE CAMERA CLOSED");
                }
            });
        } catch (Exception e) {
            System.out.println("SIDE ERROR CLOSING");
            e.printStackTrace();
        }
        openApriltag();



        position = propPipeline.propPosition;
        apriltagPipeline.pixelPosition = position;
        if(alliance == CurrentAlliance.AllianceTypes.RED)
        {
            position = 2-position;
        }
        double oldForceLimit = aCtrl.getMax_force();
        queue.addTask(new CustomTask(()->{
            aCtrl.setMax_force(80);
            return true;
        }));
        switch (position) {
            case 0:
                queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier*732,  60, 0), 0.9, ctrlHub.voltageSensor, aCtrl));
                queue.addTask(new ServoTask(purplePixelPlacer,PPP_IN));
                queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier*732,  -40, 0), 0.9, ctrlHub.voltageSensor, aCtrl));
                break;
            case 1:
                queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 974, 387, 0), 0.9, ctrlHub.voltageSensor, aCtrl));
                queue.addTask(new ServoTask(purplePixelPlacer,PPP_IN));
                queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 974, 340, 0), 0.9, ctrlHub.voltageSensor, aCtrl));
                break;
            case 2:
                queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 736, 565, 0), 1.5,ctrlHub.voltageSensor, aCtrl));
                queue.addTask(new ServoTask(purplePixelPlacer,PPP_IN));
                queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 736, 510, 0), 0.9,ctrlHub.voltageSensor, aCtrl));
//                queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 736, 545, 0), 0.9,ctrlHub.voltageSensor, aCtrl));
                //Perhaps the purple pixel peoples perfect purple pixel placer promptly placed a perfect purple pixel on the perfect purple pixel placers perfect purple pixel position pterodactyl

                break;
        }
        queue.addTask(new CustomTask(()->{
            aCtrl.setMax_force(oldForceLimit);
            return true;
        }));
        queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 480, 520, 0), 1.5,ctrlHub.voltageSensor, aCtrl));
        queue.addTask(new ServoTask(dropdownServo,DROPDOWN_ROLLER_FULL_EXTEND));
        queue.addTask(new BrakeTask(200,aCtrl));
        queue.addTask(new DelayTask(150));
        touchOffY(new Pose(0.0,250,0.0),50.0,565,1000,true);
        queue.addTask(new ServoTask(dropdownServo,DROPDOWN_ROLLER_HALF_EXTEND));
        queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 855, 550, 0), 1.5,ctrlHub.voltageSensor, aCtrl,false,true,20.0,100.0));
        queue.addTask(new ServoTask(dropdownServo,DROPDOWN_ROLLER_RETRACTED));
        safe_intake();
        queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 855, 530, 0), 1.0,ctrlHub.voltageSensor, aCtrl,true,false,15.0,100.0));
        queue.addTask(new BrakeTask(50,aCtrl));
        queue.addTask(waitForPickles(750));
        cycle(false);
        if(secondCycleState != BarfState.NONE)
        {
            cycle(secondCycleState == BarfState.YEET);
        }
        queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * (1295 ), -2000, 0), 1.0,ctrlHub.voltageSensor, aCtrl));
    }

    public void cycle(boolean barf)
    {

        FollowLineTask driveOverPath = new FollowLineTask(aCtrl,ctrlHub.voltageSensor);
        driveOverPath.addPoint(new Pose(-flipMultiplier * 855, 530, 0),0,100,false);
        driveOverPath.addPoint(new Pose(-flipMultiplier * 1295, 530, 0),0,100,false);
        driveOverPath.addPoint(new Pose(-flipMultiplier * 1295, -1940, 0),0,100);

        FollowLineTask returnPath = new FollowLineTask(aCtrl,ctrlHub.voltageSensor);

        if(compileCycle == 0)
        {
            switch (position) {
                case 0:
                    driveOverPath.addPoint(new Pose(-flipMultiplier * (550), -1940, 0));
                    driveOverPath.addPoint(new Pose(-flipMultiplier * (550), -2100, 0));
                    returnPath.addPoint(new Pose(-flipMultiplier    * (550), -2100, 0));
                    returnPath.addPoint(new Pose(-flipMultiplier    * (550), -1850, 0));
                    break;
                case 1:
                    driveOverPath.addPoint(new Pose(-flipMultiplier * 700, -1940, 0));
                    driveOverPath.addPoint(new Pose(-flipMultiplier * 700, -2100, 0));
                    returnPath.addPoint(new Pose(-flipMultiplier    * 700, -2100, 0));
                    returnPath.addPoint(new Pose(-flipMultiplier    * 700, -1850, 0));
                    break;
                case 2:
                    driveOverPath.addPoint(new Pose(-flipMultiplier * (852), -1940, 0));
                    driveOverPath.addPoint(new Pose(-flipMultiplier * (852), -2100, 0));
                    returnPath.addPoint(new Pose(-flipMultiplier    * (852), -2100, 0));
                    returnPath.addPoint(new Pose(-flipMultiplier    * (852), -1850, 0));
                    //Perhaps the perfect purple pixel peoples perfect purple pixel placer promptly placed a perfect purple pixel on the perfect purple pixel placers perfect purple pixel position pterodactyl

                    break;
            }
        }
        else
        {
            if(!barf)
            {
                driveOverPath.addPoint(new Pose(-flipMultiplier * (880 ), -1940, 0));
                driveOverPath.addPoint(new Pose(-flipMultiplier * (880 ), -2100, 0));
                returnPath.addPoint(new Pose(-flipMultiplier * (880 ), -2100, 0));
                returnPath.addPoint(new Pose(-flipMultiplier * (880 ), -1850, 0));
            }
            else
            {
                driveOverPath.addPoint(new Pose(-flipMultiplier * 1295, -1940, 0),0,100);
            }
        }
        driveOverPath.setPre_switch_accel(4.0);
        driveOverPath.setSide_accel(0.65);
        driveOverPath.setSpeed(1900);

        returnPath.addPoint(new Pose(-flipMultiplier * 1295, -1800, 0),0.0,120.0,true);
        returnPath.addPoint(new Pose(-flipMultiplier * 1295,   -200, 0),0.0,120);
        returnPath.addPoint(new Pose(-flipMultiplier * 1295,   450, 0),0.0,120);
        returnPath.setPre_switch_accel(4.0);
        returnPath.setSide_accel(0.65);
        returnPath.setSpeed(1900);


        TaskList transferList = new TaskList();
        transferList.addTask(new CustomTask(()->{
            failedTransfers = 0;
            return true;
        }));
        transfer(transferList,compileCycle>0);

        queue.addTask(transferList);
        queue.addTask(new CustomTask(()->{
            System.out.println("Doing Auto Delay");
            return (System.currentTimeMillis()-startTime) >= autoDelay;
        }));
        queue.addTask(new CustomTask(()->{
            apriltagPipeline.startPixelSampling();
            return true;
        }));
        queue.addTask(new ForkTask(driveOverPath,transferQueue));

        long startTime = System.currentTimeMillis();

        queue.addTask(new TimeoutTask(new CustomTask(()->{
            System.out.println("Apriltag Cycle Number: "+ cycleNumber);
            System.out.println("Apriltag Samples: "+ apriltagPipeline.pixelSamples);
            if(!apriltagPipeline.running)
            {
                System.out.println("ALL THE apriltags ARE DEAD WITH POISONOUS GASSES");
                return true;
            }
            if(cycleNumber != 0)
            {
                System.out.println("No Apriltag Second Cycle");
                return true;
            }
            if(apriltagPipeline.pixelSamples > 4)
            {
                apriltagPipeline.doPixelSampling = false;
                System.out.println("Apriltags presentScore: "+apriltagPipeline.presentScore);
                System.out.println("Apriltags sideScore: "+apriltagPipeline.sideScore);
                if(apriltagPipeline.presentScore < 0)
                {
                    liftMotor.setPosition(LIFT_BOARD_MIN);
                    if(position == 2)
                    {
                        deliveryRollServo.goToPreset(3);
                    }
                }
                else
                {
                    if(apriltagPipeline.sideScore*flipMultiplier < 0.0)
                    {
                        deliveryRollServo.goToPreset(3);
                    }
                }
                return true;
            }
            return false;
        }),2000));

//        queue.addTask(new MotorPositionTask(LIFT_MID, liftMotor,false,0.15f));
//        queue.addTask(new MotorPositionTask(ARM_DELIVER, armMotor,true,0.15f));
        queue.addTask(deliver(compileCycle>0,barf));
        if(compileCycle < 1 && secondCycleState != BarfState.NONE)
        {
            queue.addTask(returnPath);
            queue.addTask(new BrakeTask(500,aCtrl));
//        touchOffTruss(new Pose(flipMultiplier*300,0.0,0.0),60.0,-1225,400,1400,true);
            queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 1300, 450, 0), 1.0,ctrlHub.voltageSensor, aCtrl,false,false,12.0,150.0));
            queue.addTask(new ServoTask(dropdownServo,DROPDOWN_ROLLER_FULL_EXTEND));
            touchOffY(new Pose(0.0,250,0.0),50.0,565,1000,true);
            if(compileCycle == 1)
            {
                queue.addTask(new ServoTask(dropdownServo,DROPDOWN_ROLLER_FULL_EXTEND));
            }
            else
            {
                queue.addTask(new ServoTask(dropdownServo,DROPDOWN_ROLLER_FULL_EXTEND));
            }
            queue.addTask(new DelayTask(75));
            queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 1050, 550, 0), 1.0,ctrlHub.voltageSensor, aCtrl,false,true,20.0,100.0));
            queue.addTask(new ServoTask(dropdownServo,DROPDOWN_ROLLER_RETRACTED));
            queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 1300+isBlue*30, 550, 0), 1.0,ctrlHub.voltageSensor, aCtrl,false,true,12.0,100.0));
            safe_intake();
            queue.addTask(new SmoothPointTask(new Pose(-flipMultiplier * 1300+isBlue*30, 550, 0), 1.0,ctrlHub.voltageSensor, aCtrl,true,false,12.0,100.0));
            queue.addTask(new BrakeTask(50,aCtrl));
            queue.addTask(waitForPickles(1000));
        }
        queue.addTask(new CustomTask(()->{
            cycleNumber++;
            return true;
        }));
        compileCycle++;
    }

    public TaskList deliver(boolean vertical,boolean barf)
    {
        TaskList out = new TaskList();

        out.addTask(new TimeoutTask(new CustomTask(()->{
            if(cycleNumber >= 1)
            {
                return true;
            }
            return false;
        }),300));
        if(vertical)
        {
            out.addTask(new ServoTask(deliveryRollServo,DELIVERY_ROLL_LOW+(DELIVERY_ROLL_HIGH-DELIVERY_ROLL_LOW)*4.5/5.0));
        }

        if(!barf)
        {
            out.addTask(boardPush(new Pose(0.0,-500,0.0),50.0,-2190,2500));
        }


        if(alliance == CurrentAlliance.AllianceTypes.BLUE)
        {
            out.addTask(new ServoTask(pincherRightServo,PINCHER_RIGHT_OPEN));
            if(vertical)
            {
                out.addTask(new DelayTask(500));
            }
            out.addTask(new ServoTask(pincherLeftServo,PINCHER_LEFT_OPEN));
        }
        else
        {
            out.addTask(new ServoTask(pincherLeftServo,PINCHER_LEFT_OPEN));
            if(vertical)
            {
                out.addTask(new DelayTask(500));
            }
            out.addTask(new ServoTask(pincherRightServo,PINCHER_RIGHT_OPEN));
        }
        out.addTask(new DelayTask(60));
        out.addTask(new CustomTask(()->{
            armCtrlr.disable();
            System.out.println("How the heck is it 0: "+armCtrlr.getPid().getMaxPower());
            armCtrlr.getPid().setMaxPower(0.3);
            return true;
        }));
        out.addTask(new DelayTask(60));
        out.addTask(new MotorPositionTask(ARM_PRE_DELIVER,armMotor,false,1.0f));
        out.addTask(new CustomTask(()->{
            armCtrlr.enable();
            return true;
        }));
        out.addTask(new MotorPositionTask(ARM_PRE_DELIVER-0.4,armMotor,true,0.5f));
        out.addTask(new ServoPresetTask(deliveryRollServo,0));
        out.addTask(new CustomTask(()->{
            deliveryRollServo.goToPreset(0);
            armCtrlr.getPid().setMaxPower(1.0);
            return true;
        }));
        out.addTask(new DelayTask(50));
        out.addTask(new MotorPositionTask(ARM_TRANSFER,armMotor,false,0.5f));
        out.addTask(new MotorPositionTask(0,liftMotor,false,0.05f));
        out.addTask(new MotorPowerTask(0.0,intakeMotor));
        return out;
    }
    TaskList boardPush(Pose velocity, double maxForce, double finalPosition, int timeout)
    {
        TaskList out = new TaskList();
        out.addTask(new CustomTask(()->{
            aCtrl.setMax_force(maxForce);
            aCtrl.setVelocity(velocity,ctrlHub.voltageSensor.getVoltage(),new Pose(0,0,0));
            return true;
        }));
        out.addTask(new TimeoutTask(new CustomTask(()->{
            if(armCtrlr.getPosition() > -1.15)
            {
                System.out.println("Detected Push In");
                return true;
            }
            return false;
        }),timeout));
        out.addTask(new CustomTask(()->{
            aCtrl.setMax_force(oldForceLimit);
            aCtrl.setPowerVec(new Pose(0.0,0.0,0.0));
            System.out.println("Y Position From Board! Prior Y: "+encoderLocalizer.getPos().getY());
            encoderLocalizer.getPos().setY(finalPosition);
            return true;
        }));
        return out;
    }


    void touchOffY(Pose velocity,double maxForce, double finalPosition, int timeout , boolean doReset)
    {
        AtomicBoolean pushStarted = new AtomicBoolean(false);
        // Side touch off
        queue.addTask(new CustomTask(()->{
            aCtrl.setMax_force(maxForce);
            aCtrl.setVelocity(velocity,ctrlHub.voltageSensor.getVoltage(),new Pose(0,0,0));
            pushStarted.set(false);
            return true;
        }));
        queue.addTask(new TimeoutTask(new CustomTask(()->{
            if(Math.abs(encoderLocalizer.getLocalVelocity().getY()) > 150)
            {
                pushStarted.set(true);
                System.out.println("Y Push Started");
            }
            if(Math.abs(encoderLocalizer.getLocalVelocity().getY()) < 20 && pushStarted.get())
            {
                System.out.println("Detected STop");
                return true;
            }
            return false;
        }),timeout));
        queue.addTask(new CustomTask(()->{
            aCtrl.setMax_force(oldForceLimit);
            aCtrl.setPowerVec(new Pose(0.0,0.0,0.0));
            if(doReset)
            {
                System.out.println("Y Position Homed! Prior Y: "+encoderLocalizer.getPos().getY());
                encoderLocalizer.getPos().setY(finalPosition);
            }
            return true;
        }));

    }
    void touchOffTruss(Pose velocity,double maxForce, double finalPosition, int stalled_timeout, int timeout , boolean doReset)
    {
        AtomicBoolean pushStarted = new AtomicBoolean(false);
        AtomicLong initialTime = new AtomicLong();
        // Side touch off
        queue.addTask(new CustomTask(()->{
            aCtrl.setMax_force(maxForce);
            velocity.setR(-3.0*encoderLocalizer.getPos().getR());
            System.out.println("Velocity: "+velocity);
            aCtrl.setVelocity(velocity,ctrlHub.voltageSensor.getVoltage(),new Pose(0,0,0));
            pushStarted.set(false);
            initialTime.set(System.currentTimeMillis());
            return true;
        }));
        queue.addTask(new TimeoutTask(new CustomTask(()->{
            velocity.setR(-5.0*encoderLocalizer.getPos().getR());
            System.out.println("Velocity: "+velocity);
            aCtrl.setVelocity(velocity,ctrlHub.voltageSensor.getVoltage(),new Pose(0,0,-3.0*encoderLocalizer.getPos().getR()));
            if(Math.abs(encoderLocalizer.getLocalVelocity().getX()) > 100)
            {
                pushStarted.set(true);
                System.out.println("X Push Started");
            }
            if(Math.abs(encoderLocalizer.getLocalVelocity().getX()) < 50 && (pushStarted.get() || System.currentTimeMillis()-initialTime.get() > stalled_timeout))
            {
                System.out.println("Detected STop InitialTime: "+initialTime.get()+" ,CurrentTime: "+(System.currentTimeMillis()-initialTime.get()));
                return true;
            }
            return false;
        }),timeout));
        queue.addTask(new CustomTask(()->{
//            aCtrl.setMax_force(oldForceLimit);
            aCtrl.setPowerVec(new Pose(0.0,0.0,0.0));
            if(doReset)
            {
                System.out.println("X Position Homed To Truss ! Prior X: "+encoderLocalizer.getPos().getX());
                encoderLocalizer.getPos().setX(finalPosition+100*Math.abs(Math.sin(encoderLocalizer.getPos().getR()))*Math.signum(velocity.getX()));
            }
            return true;
        }));
    }

    public void transfer(TaskList out,boolean verticalPixel) {
        TaskList transferList = new TaskList();


        transferList.addTask(new ServoTask(fingersServo, FINGERS_UP));
        transferList.addTask(new MotorPositionTask(ARM_TRANSFER, armMotor, true, 0.08f));
        transferList.addTask(new DelayTask(250));
        transferList.addTask(new MotorPowerTask(0.0, intakeMotor));
        transferList.addTask(new ServoTask(trayServo, TRAY_TRANSFER));
        transferList.addTask(new DelayTask(250));
        transferList.addTask(new MotorPowerTask(0.65, intakeMotor));
        transferList.addTask(new CustomTask(()->{
            transferQueue.addTask(new ServoTask(pincherLeftServo, PINCHER_LEFT_CLOSED));
            transferQueue.addTask(new ServoTask(pincherRightServo, PINCHER_RIGHT_CLOSED));
            transferQueue.addTask(new DelayTask(50));

            transferQueue.addTask(new ServoTask(fingersServo, FINGERS_DOWN));
            transferQueue.addTask(new DelayTask(50));
            transferQueue.addTask(new MotorPositionTask(ARM_PRE_DELIVER, armMotor, false, 0.08f));
            transferQueue.addTask(new DelayTask(350));

            transferQueue.addTask(new MotorPowerTask(0.0, intakeMotor));


            transferQueue.addTask(new MotorPositionTask(ARM_PRE_DELIVER, armMotor,true,0.15f));
            transferQueue.addTask(new ServoTask(trayServo, TRAY_INTAKE));

            transferQueue.addTask(new CustomTask(() -> {
                return encoderLocalizer.getPos().getY() < -1400;
            }));

            transferQueue.addTask(new CustomTask(() -> {
                if (cycleNumber == 0) {
                    liftMotor.setPosition(LIFT_MID + 20);
                } else {
                    liftMotor.setPosition(LIFT_MID + 120);
                }
                return true;
            }));
            transferQueue.addTask(new MotorPositionTask(ARM_DELIVER, armMotor, true, 0.15f));
            if(verticalPixel)
            {
                transferQueue.addTask(new ServoTask(deliveryRollServo,DELIVERY_ROLL_LOW+(DELIVERY_ROLL_HIGH-DELIVERY_ROLL_LOW)*4.5/5.0));
            }

            return true;
        }));
        out.addTask(new MotorPowerTask(0.0, intakeMotor));
        out.addTask(new ServoTask(pincherLeftServo, PINCHER_LEFT_OPEN));
        out.addTask(new ServoTask(pincherRightServo, PINCHER_RIGHT_OPEN));


        out.addTask(new MotorPowerTask(-1.0, intakeMotor));
        out.addTask(new ServoTask(trayServo, TRAY_PARTIAL));
        out.addTask(new DelayTask(500));
        out.addTask(new ServoTask(fingersServo, FINGERS_UP));
        out.addTask(new DelayTask(200));
        // Set to 0.03 to brake motor as a time saving measure
        out.addTask(new MotorPowerTask(0.03, intakeMotor));
        out.addTask(new DelayTask(200));

        // TODO:
        // switch to Cuttlefish ring buffer
        int samples = 3;
        double[] fingersBuffer = new double[samples];
        AtomicBoolean isJammed = new AtomicBoolean(false);

        out.addTask(new CustomTask(()->{
            double fingersAv = 0;
            double fingersDev = 0;
            double fingersMax = 0;
            for(int i = samples-1; i > 0; i--)
            {
                fingersBuffer[i] = fingersBuffer[i-1];
                fingersAv+= fingersBuffer[i-1];
            }
            fingersBuffer[0] = fingersEncoder.getVoltage()/1000.0;
            fingersAv +=  fingersBuffer[0];
            fingersAv /= samples;

            for(int i = 0; i < samples; i++)
            {
                fingersDev += Math.pow(fingersBuffer[i]-fingersAv,2);
                fingersMax = Math.max(fingersBuffer[i],fingersMax);
                System.out.println("BufferVal: "+fingersBuffer[i]);
            }
            fingersDev /= samples;
            System.out.println("Sampling! FingersDev: "+fingersDev+", fingersMax: "+fingersMax);

            if(fingersDev < 0.0003)
            {
                isJammed.set(fingersMax > 1.436);
                return true;
            }
            return false;
        }));
        out.addTask(new CustomTask(()->{
            if(!isJammed.get())
            {
                //TODO: If auto twisty to deliver yellow to board doesnt untwisty
                out.addTask(new CustomTask(() -> {
                    boolean trayLeftTouch = !trayPixelSwitchLeft.getState();
                    boolean trayRightTouch = !trayPixelSwitchRight.getState();
                    System.out.println("Tray Left Touch: " + trayLeftTouch);
                    System.out.println("Tray Right Touch: " + trayRightTouch);
                    if (!trayLeftTouch) {
                        if (!leftFilled) {
                            double leftDist = leftColorSensor.getDistance(DistanceUnit.MM);
                            leftFilled = leftDist < LEFT_COLOR_SENSOR_MAX_THRESHOLD;
                            if (!leftFilled && leftDist != 0) {
                                trayLeftTouch = true;
                            }
                            System.out.println("Tray Left Touch Post Sensor: " + trayLeftTouch + ", leftDist: " + leftDist);
                        }
                    }
                    if (!trayRightTouch) {
                        if (!rightFilled) {
                            double rightDist = rightColorSensor.getDistance(DistanceUnit.MM);
                            rightFilled = rightDist < RIGHT_COLOR_SENSOR_MAX_THRESHOLD;
                            if (!rightFilled && rightDist != 0) {
                                trayRightTouch = true;
                            }
                            System.out.println("Tray Right Touch Post Sensor: " + trayRightTouch + ", rightDist: " + rightDist);
                        }
                    }


                    if (trayLeftTouch && trayRightTouch) {
                        out.addTask(transferList);
                    } else {
                        out.addTask(new ServoTask(fingersServo, FINGERS_DOWN));
                        out.addTask(new DelayTask(200));
                        out.addTask(new MotorPowerTask(0.5, intakeMotor));
                        out.addTask(new DelayTask(100));
                        out.addTask(new MotorPowerTask(-1.0, intakeMotor));
                        if (!atBoard) {
                            transfer(out, verticalPixel);
                        }
                        else {
                            out.addTask(new ServoTask(trayServo, TRAY_INTAKE));
                        }
                    }
                    return true;
                }));
            }
            else {
                out.addTask(new MotorPowerTask(1.0, intakeMotor));
                out.addTask(new DelayTask(600));
                out.addTask(new ServoTask(fingersServo, FINGERS_DOWN));
                out.addTask(new ServoTask(trayServo, TRAY_INTAKE));
                if (!atBoard) {
                    transfer(out, verticalPixel);
                }
                else {
                    out.addTask(new ServoTask(trayServo, TRAY_INTAKE));
                }
            }
            return true;
        }));
    }
    TaskList delayedArmDelivery()
    {
        TaskList out = new TaskList();
        out.addTask(new CustomTask(() -> {
            return encoderLocalizer.getPos().getY() < -1400;
        }));

        out.addTask(new CustomTask(() -> {
            if (cycleNumber == 0) {
                liftMotor.setPosition(LIFT_MID + 20);
            } else {
                liftMotor.setPosition(LIFT_MID + 120);
            }
            return true;
        }));
        out.addTask(new MotorPositionTask(ARM_DELIVER, armMotor, true, 0.15f));
        return out;
    }

    Task waitForPickles(int timeout)
    {
        TaskList unjammer = new TaskList();
        TaskList out = new TaskList();
        AtomicBoolean unjamming = new AtomicBoolean(false);
        out.addTask(new TimeoutTask(new CustomTask(()->{
            double current = intakeMotor.getCurrent()/1000.0;
            if(current > 7.5 && !unjamming.get())
            {
                unjamming.set(true);
                unjammer.addTask(new MotorPowerTask(0.6,intakeMotor));
                unjammer.addTask(new DelayTask(300));
                unjammer.addTask(new MotorPowerTask(-1.0,intakeMotor));
                unjammer.addTask(new CustomTask(()->{
                    unjamming.set(false);
                    return true;
                }));
            }
            if(!leftFilled)
            {
                double leftDist = leftColorSensor.getDistance(DistanceUnit.MM);
                if(leftDist<LEFT_COLOR_SENSOR_MAX_THRESHOLD)
                {
                    leftFilled = true;
                }
                System.out.println("Left Pickle Dist: "+leftDist);
            }
            else if(!rightFilled)
            {
                double rightDist = rightColorSensor.getDistance(DistanceUnit.MM);
                if(rightDist<RIGHT_COLOR_SENSOR_MAX_THRESHOLD)
                {
                    rightFilled = true;
                }
                System.out.println("Right Pickle Dist: "+rightDist);
            }
            else
            {
                System.out.println("Both Pickles Detected");
                return true;
            }
            unjammer.loop();
            return false;
        }),timeout));


        return out;
    }
    void safe_intake()
    {
        queue.addTask(new MotorPowerTask(-1.0,intakeMotor));
        touchOffY(new Pose(0.0,600,0.0),40,0,700,false);
        queue.addTask(new CustomTask(()->{
            leftFilled = false;
            rightFilled = false;
            return true;
        }));

        queue.addTask(waitForPickles(750));

    }
    @Override
    public void mainLoop() {
        super.mainLoop();
        if(elapsedTime.time()>29700)
        {
            queue.clear();
            if(armCtrlr.getPosition()<-0.5)
            {
                pincherLeftServo.setPosition(PINCHER_LEFT_OPEN);
                pincherRightServo.setPosition(PINCHER_RIGHT_OPEN);
            }
            else
            {
                armCtrlr.enable();
                armCtrlr.setPosition(-0.6);
            }
            armCtrlr.getPid().setMaxPower(1.0);

            Pose brakeDir = encoderLocalizer.getLocalVelocity().clone();
            brakeDir.setR(0.0);
            brakeDir.normalize();
            brakeDir.scale(-0.05,false);
            aCtrl.setPowerVec(brakeDir);
        }
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("XPos: ",encoderLocalizer.getPos().getX());
        packet.put("YPos: ",encoderLocalizer.getPos().getY());
        if(!twoEncoderLocalizer)
        {
            packet.put("LEnc: ",((ThreeEncoderLocalizer)encoderLocalizer).getL().getRotation());
            packet.put("REnc: ",((ThreeEncoderLocalizer)encoderLocalizer).getR().getRotation());
        }
        else
        {
            packet.put("FEnc: ",((TwoEncoderLocalizer)encoderLocalizer).getF());
        }
        packet.put("RPos: ",encoderLocalizer.getPos().getR());
        packet.put("dT", (float)dT/(1000.0*1000.0));
        dash.sendTelemetryPacket(packet);
    }
    public void onEnd()
    {
        super.onEnd();
        try {
//            apriltagWebcam.stopStreaming();
            apriltagWebcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener(){
                @Override
                public void onClose() {
                    System.out.println("APRILTAG CAMERA CLOSED");
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}

