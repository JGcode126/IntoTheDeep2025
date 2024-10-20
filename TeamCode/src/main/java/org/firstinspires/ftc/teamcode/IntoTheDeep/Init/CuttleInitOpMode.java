package org.firstinspires.ftc.teamcode.IntoTheDeep.Init;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleTestSlides;
import org.firstinspires.ftc.teamcode.Testing.SparkFunOTOS;


//@Disabled
@Config
public abstract class CuttleInitOpMode extends GamepadOpMode {
    // Declare the rev hubs. If you only have one hub connected you can delete one of these
    public CuttleRevHub ctrlHub;
    public CuttleRevHub expHub;

    public CuttleDT dt;
    public CuttleTestSlides ts;
    public CuttleExtendo extendo;
    public  CuttleIntake intake;
    public CuttleSlides lift;

    // Declare the chassis motors
    public CuttleMotor leftFrontMotor;
    public CuttleMotor rightFrontMotor;
    public CuttleMotor rightBackMotor;
    public CuttleMotor leftBackMotor;

    public CuttleMotor leftbackSlides;
    public CuttleMotor rightBackSlides;
    public CuttleMotor extendoMotor;

    public CuttleMotor testSlides;


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
    SparkFunOTOS.Pose2D pos;
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

        ctrlHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.CONTROL_HUB);
        expHub = new CuttleRevHub(hardwareMap,"Expansion Hub 2");
        //myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

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

        leftbackSlides  = ctrlHub.getMotor(3);
        rightBackSlides = expHub.getMotor(3);
        leftbackSlides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackSlides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        CuttleEncoder liftEncoder = expHub.getEncoder(3, 141.1*4);
        extendoMotor = ctrlHub.getMotor(2);
        CuttleEncoder extendoEncoder = ctrlHub.getEncoder(2, 141.1*4);
        extendoMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        //CuttleEncoder testEncoder = expHub.getEncoder(2, 384.5);
        CuttleServo intakeLeft = expHub.getServo(0);
        CuttleServo intakeRight = ctrlHub.getServo(5);
        CuttleServo intakeClaw = ctrlHub.getServo(0);
        CuttleServo turntable = ctrlHub.getServo(1);

        //Initialize and set the direction of the encoders
        CuttleEncoder leftEncoder = ctrlHub.getEncoder(1,2000);
        CuttleEncoder sideEncoder = ctrlHub.getEncoder(2,2000);
        CuttleEncoder rightEncoder = ctrlHub.getEncoder(0,2000);
        rightEncoder.setDirection(Direction.REVERSE);
        sideEncoder.setDirection(Direction.REVERSE);

        // Initialize the mecanum controller
        chassis = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);

        // Initialize the localizer
        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  , // Left
                sideEncoder  , // Side
                rightEncoder , // Right
                24,
                395,
                0.988
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
    }
    @Override
    public void main() {
    }
    public void mainLoop()
    {
        super.mainLoop();

        //pos = myOtos.getPosition();
        // Pull bulk data from both hubs
        ctrlHub.pullBulkData();
        expHub.pullBulkData();

        // Update the localizer
        encoderLocalizer.update();

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
}