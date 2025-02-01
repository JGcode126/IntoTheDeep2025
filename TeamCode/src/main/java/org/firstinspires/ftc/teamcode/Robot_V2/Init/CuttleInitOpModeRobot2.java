package org.firstinspires.ftc.teamcode.Robot_V2.Init;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefish.utils.PID;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed.AutoSequence;
import org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed.Battery;
import org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed.BucketAuto;
import org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed.Setup;
import org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed.SpecimenAuto;
import org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed.TaskManager;
import org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed.TeleOp;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleDT;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleHang;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides;
import org.firstinspires.ftc.teamcode.Testing.SparkFunOTOS;


//@Disabled
@Config
public abstract class CuttleInitOpModeRobot2 extends GamepadOpMode {
    // Declare the rev hubs. If you only have one hub connected you can delete one of these
    public CuttleRevHub ctrlHub;
    public CuttleRevHub expHub;

    public v2CuttleDT dt;
    public v2CuttleExtendo extendo;
    public v2CuttleIntake intake;
    public v2CuttleSlides lift;
    public v2CuttleOutake outake;
    public v2CuttleHang hang;

    // Declare the chassis motors
    public CuttleMotor leftFrontMotor;
    public CuttleMotor rightFrontMotor;
    public CuttleMotor rightBackMotor;
    public CuttleMotor leftBackMotor;

    public CuttleMotor leftbackSlides;
    public CuttleMotor rightBackSlides;
    public CuttleMotor extendoMotor;

    // Declare the mecanum controller
    public MecanumController chassis;

    // Declare the localizer
    public ThreeEncoderLocalizer encoderLocalizer;
    public ThreeEncoderLocalizer otosLocalizer;

    public MotorPositionController liftPosController;
    public MotorPositionController extendoPosController;

    // Declare the PTPController
    public PTPController ptpController;
    public PTPController ptpOdoController;

    // Declare the task queue
    public TaskQueue queue;
    public SparkFunOTOS.Pose2D pos;
    SparkFunOTOS myOtos;

    public Setup setup;
    public AutoSequence auto;
    public SpecimenAuto specimen;
    public BucketAuto bucket;
    public Battery battery;

    public CuttleEncoder leftEncoder, sideEncoder, rightEncoder;


    public String color = null;

    //public static int method;
    public static double extendoPosition;
    public static double liftPosition;

    @Override
    public void onInit()
    {
        //rev hubs
        ctrlHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.CONTROL_HUB);
        expHub = new CuttleRevHub(hardwareMap,"Expansion Hub 2");

        int batteryVoltage = 14;
        double optimalVoltage = 13.9;



        //otos
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        //check if otos is plugged
        try {
            SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
            SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
            myOtos.getVersionInfo(hwVersion, fwVersion);

            if (hwVersion.major == 0 && hwVersion.minor == 0 && fwVersion.major == 0 && fwVersion.minor == 0) {
                throw new Exception("OTOS not responding! Check connection.");
            }
        }
        catch (Exception e) {
            color = "red";
        }

        //drivetrain
        leftFrontMotor  = ctrlHub.getMotor(1);
        leftBackMotor   = ctrlHub.getMotor(0);
        rightFrontMotor = expHub.getMotor(1);
        rightBackMotor  = expHub.getMotor(0);
        leftFrontMotor.setDirection(Direction.REVERSE);
        leftBackMotor.setDirection(Direction.REVERSE);
        leftFrontMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        //lift slides
        leftbackSlides  = ctrlHub.getMotor(2);
        rightBackSlides = expHub.getMotor(2);
        leftbackSlides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackSlides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        CuttleEncoder liftEncoder = ctrlHub.getEncoder(2, 141.1*4);
        liftEncoder.setDirection(Direction.REVERSE);

        //extendo
        extendoMotor = expHub.getMotor(3);
        CuttleEncoder extendoEncoder = expHub.getEncoder(3, 141.1*4);
        extendoMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        //hang
        CuttleServo hangL = ctrlHub.getServo(5);
        CuttleServo hangR = expHub.getServo(4);

        //outtake - all others at servoHub - configure with hardware map
        CuttleServo outtakeClawServo = ctrlHub.getServo(3);

        //intake - all others at servoHub - configure with hardware map
        CuttleServo intakeClaw = ctrlHub.getServo(2);
        CuttleServo intakeTurntable = ctrlHub.getServo(4);
        CuttleServo light = expHub.getServo(5);


        //Odometry
        leftEncoder = ctrlHub.getEncoder(1,2000);
        sideEncoder = ctrlHub.getEncoder(3,2000);
        rightEncoder = expHub.getEncoder(1,2000);
        //leftEncoder.setDirection(Direction.REVERSE);
        sideEncoder.setDirection(Direction.REVERSE);


        // Initialize the mecanum controller
        chassis = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);

        // Initialize the localizer
        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  , // Left
                sideEncoder  , // Side
                rightEncoder , // Right
                17.5,
                288.92500,
                1
        );

        otosLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  , // Left
                sideEncoder  , // Side
                rightEncoder , // Right
                17.5,
                288.92500,
                0.9140322
        );

        extendoPosController = new MotorPositionController(0,extendoMotor, extendoEncoder, true);
        extendoPosController.setPid(new PID(1.7, 0, 0.04, 0,1));
        liftPosController = new MotorPositionController(0, rightBackSlides, liftEncoder, true);

        // Initialize the PTP Controller
        ptpController = new PTPController(chassis, encoderLocalizer);//for odo
        ptpOdoController = new PTPController(chassis, otosLocalizer);//for otos something that I want to try to use
        //for right now for oto
        //ptpController = new PTPController(chassis, otosLocalizer);//for odo



        /*ptpController.setTranslational_PD_ctrlr(new PID(
                0.015,0,0.002,0,1
        ));//0.00025
        ptpController.setRotational_PID_ctrlr(new PID(3.2,0.0,0,0,1));
        ptpController.getAntistallParams().setMovePowerAntistallThreshold(0.25);
        ptpController.getAntistallParams().setMoveSpeedAntistallThreshold(0.1);
        ptpController.getAntistallParams().setRotatePowerAntistallThreshold(0.25);
        ptpController.getAntistallParams().setRotateSpeedAntistallThreshold(0.1);*/

        // Initialize the queue
        queue = new TaskQueue();
        dt = new v2CuttleDT(leftBackMotor,leftFrontMotor, rightBackMotor, rightFrontMotor, expHub, ctrlHub);
        extendo = new v2CuttleExtendo(extendoMotor, extendoEncoder, extendoPosController, ctrlHub);
        intake = new v2CuttleIntake(intakeClaw, intakeTurntable, hardwareMap, light,color);
        lift = new v2CuttleSlides(leftbackSlides, rightBackSlides, liftEncoder, liftPosController,ctrlHub);
        outake = new v2CuttleOutake(outtakeClawServo, hardwareMap);
        hang = new v2CuttleHang(hangL,hangR);

        setup = new Setup(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt, hang);

        auto = new AutoSequence(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt,
                new TaskManager(queue, ptpController, batteryVoltage, optimalVoltage), new TeleOp(intake, outake,extendo,lift,dt,
                new TaskManager(queue, ptpController, batteryVoltage, optimalVoltage)), hang);

        specimen = new SpecimenAuto(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt,
                new TaskManager(queue, ptpController, batteryVoltage, optimalVoltage), hang);

        bucket = new BucketAuto(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt,
                new TaskManager(queue, ptpController,batteryVoltage, optimalVoltage), hang);


        battery = new Battery(batteryVoltage, optimalVoltage);
        configureOtos();
    }
    @Override
    public void main() {
    }
    public void mainLoop() {
        super.mainLoop();

        pos = myOtos.getPosition();
        // Pull bulk data from both hubs
        ctrlHub.pullBulkData();
        expHub.pullBulkData();

        // Update the localizer
        encoderLocalizer.update();

        //otosLocalizer.setPos(new Pose(pos.x, pos.y, pos.h)); //Using otos

        lift.setLiftPosition(liftPosition);
        extendo.setSlidePosition(extendoPosition);
/*
        if (intake.intakeState != LOOKING && !gamepad1.share) {
            extendo.setSlidePosition(extendoPosition);
        }
        if (intake.intakeState == LOOKING && !gamepad1.share){
            extendo.setSlidePositionColor(extendoPosition);

        }

 */
        //extendoPosController.setPosition(extendoPosition);


        // Update the queue
        queue.update();
        //extendoPosController.loop();

    }

    public double[] sensorFusion(double otosChange, double odoChange){
        double otosR = myOtos.getPosition().h;
        double otosX = myOtos.getPosition().x;
        double otosY = myOtos.getPosition().y;

        double odometryR = encoderLocalizer.getPos().getR();
        double odometryX = encoderLocalizer.getPos().getX();
        double odometryY = encoderLocalizer.getPos().getY();

        double fusionR = (otosR * otosChange) + (odometryR * odoChange);
        double fusionX = (otosX * otosChange) + (odometryX * odoChange);
        double fusionY = (otosY * otosChange) + (odometryY * odoChange);

        return new double[]{fusionX,fusionY,fusionR};
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.MM);
        myOtos.setAngularUnit(AngleUnit.RADIANS);
        //myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        pos = myOtos.getPosition();

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.addData("Heading", pos.h);
        telemetry.update();
    }
}