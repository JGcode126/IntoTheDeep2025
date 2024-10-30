package org.firstinspires.ftc.teamcode.IntoTheDeep.Init;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;
import org.firstinspires.ftc.teamcode.Testing.SparkFunOTOS;


//@Disabled
@Config
public abstract class CuttleInitOpMode extends GamepadOpMode {
    // Declare the rev hubs. If you only have one hub connected you can delete one of these
    public CuttleRevHub ctrlHub;
    public CuttleRevHub expHub;

    public CuttleDT dt;
    public CuttleExtendo extendo;
    public CuttleIntake intake;
    public CuttleSlides lift;
    public CuttleOutake outake;

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

    public MotorPositionController liftPosController;
    public MotorPositionController extendoPosController;

    // Declare the PTPController
    public PTPController ptpController;

    // Declare the task queue
    public TaskQueue queue;
    public SparkFunOTOS.Pose2D pos;
    SparkFunOTOS myOtos;
/*
    public static double p;
    public static double i;
    public static double d;

    public static double pRotation;
    public static double iRotation;
    public static double dRotation;

 */

    //public static int method;
    public static double extendoPosition;
    public static double liftPosition;

    @Override
    public void onInit()
    {
        //rev hubs
        ctrlHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.CONTROL_HUB);
        expHub = new CuttleRevHub(hardwareMap,"Expansion Hub 2");

        //otos
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

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
        leftbackSlides  = ctrlHub.getMotor(3);
        rightBackSlides = expHub.getMotor(3);
        leftbackSlides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackSlides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        CuttleEncoder liftEncoder = expHub.getEncoder(3, 141.1*4);

        //extendo
        extendoMotor = ctrlHub.getMotor(2);
        CuttleEncoder extendoEncoder = ctrlHub.getEncoder(2, 141.1*4);
        extendoMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake
        CuttleServo intakeLeft = expHub.getServo(0);
        CuttleServo intakeRight = ctrlHub.getServo(5);
        CuttleServo intakeClaw = ctrlHub.getServo(0);
        CuttleServo turntable = ctrlHub.getServo(1);

        //outake
        CuttleServo driveServo = expHub.getServo(3);
        CuttleServo clawServo = expHub.getServo(5);
        CuttleServo wristServo = expHub.getServo(4);


        //Odometry
        CuttleEncoder leftEncoder = ctrlHub.getEncoder(1,2000);
        CuttleEncoder sideEncoder = expHub.getEncoder(0,2000);
        CuttleEncoder rightEncoder = expHub.getEncoder(1,2000);
        leftEncoder.setDirection(Direction.REVERSE);

        // Initialize the mecanum controller
        chassis = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);

        // Initialize the localizer
        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  , // Left
                sideEncoder  , // Side
                rightEncoder , // Right
                17.5,
                288.92500,
                0.9140322
        );

        extendoPosController = new MotorPositionController(0,extendoMotor, extendoEncoder, true);
        liftPosController = new MotorPositionController(0, rightBackSlides, liftEncoder, true);

        // Initialize the PTP Controller
        ptpController = new PTPController(chassis, encoderLocalizer);

        /*ptpController.setTranslational_PD_ctrlr(new PID(
                p,i, d,2.0/1000.0,0
        ));//0.02, 0, 0.0005*/
        //ptpController.setRotational_PID_ctrlr(new PID(pRotation,iRotation,dRotation,0.0,0.35));
        //PI * 1, 0, 0.2

       /* ptpController.setTranslational_PD_ctrlr(new PID(
                0.002,0, 0.0002,2.0/1000.0,0
        ));//0.02, 0, 0.0005
        ptpController.setRotational_PID_ctrlr(new PID(0.9,0,0.25,0.0,0.35));*/
        //PI * 1, 0, 0.2
        //50%-50% = 2,0,0.2
        //20%-80% = 0.9,0, 0.25

        /*ptpController.getAntistallParams().setMovePowerAntistallThreshold(0.2);
        ptpController.getAntistallParams().setMoveSpeedAntistallThreshold(.05);
        ptpController.getAntistallParams().setRotatePowerAntistallThreshold(0.05);
        ptpController.getAntistallParams().setRotateSpeedAntistallThreshold(0.05);*/

        // Initialize the queue
        queue = new TaskQueue();
        dt = new CuttleDT(leftBackMotor,leftFrontMotor, rightBackMotor, rightFrontMotor, expHub, ctrlHub);
        extendo = new CuttleExtendo(extendoMotor, extendoEncoder, extendoPosController, ctrlHub);
        intake = new CuttleIntake(intakeLeft, intakeRight, intakeClaw, turntable, hardwareMap);
        lift = new CuttleSlides(leftbackSlides, rightBackSlides, liftEncoder, liftPosController,ctrlHub);
        outake = new CuttleOutake(driveServo, wristServo, clawServo);
        configureOtos();
    }
    @Override
    public void main() {
    }
    public void mainLoop()
    {
        super.mainLoop();

        pos = myOtos.getPosition();
        // Pull bulk data from both hubs
        ctrlHub.pullBulkData();
        expHub.pullBulkData();

        // Update the localizer
        //encoderLocalizer.update();

        encoderLocalizer.setPos(new Pose(pos.x, pos.y, pos.h)); //Using otos

        extendo.setSlidePosition(extendoPosition);
        lift.setLiftPosition(liftPosition);

        // Update the queue
        queue.update();
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