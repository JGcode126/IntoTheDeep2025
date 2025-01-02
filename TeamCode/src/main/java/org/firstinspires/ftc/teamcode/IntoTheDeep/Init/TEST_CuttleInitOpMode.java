package org.firstinspires.ftc.teamcode.IntoTheDeep.Init;


import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.LOOKING;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefish.utils.PID;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.AutoSequence;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.RegularlyUsedBucketAuto;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.RegularlyUsedSpecimenAuto;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.Setup;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.TaskManager;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed.TeleOp;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;
import org.firstinspires.ftc.teamcode.Testing.SparkFunOTOS;


//@Disabled
@Config
public abstract class TEST_CuttleInitOpMode extends GamepadOpMode {
    // Declare the rev hubs. If you only have one hub connected you can delete one of these
    public CuttleRevHub ctrlHub;

    // Declare the localizer
    public ThreeEncoderLocalizer encoderLocalizer;

    // Declare the PTPController
    public PTPController ptpController;
    public PTPController ptpOdoController;

    // Declare the task queue
    public TaskQueue queue;

    public Limelight3A limelight;

    @Override
    public void onInit() {
        //rev hubs
        ctrlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);


        //otos
        /*myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

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
        }*/


        //Odometry
        CuttleEncoder leftEncoder = ctrlHub.getEncoder(1, 2000);
        CuttleEncoder sideEncoder = ctrlHub.getEncoder(0, 2000);
        CuttleEncoder rightEncoder = ctrlHub.getEncoder(1, 2000);
        leftEncoder.setDirection(Direction.REVERSE);

        // Initialize the localizer
        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder, // Left
                sideEncoder, // Side
                rightEncoder, // Right
                17.5,
                288.92500,
                0.9140322
        );

    }
    @Override
    public void main() {
    }
    public void mainLoop() {
        super.mainLoop();

        // Pull bulk data from both hubs
        ctrlHub.pullBulkData();

        // Update the localizer
        encoderLocalizer.update();

        // Update the queue
        queue.update();
        //extendoPosController.loop();

    }
}