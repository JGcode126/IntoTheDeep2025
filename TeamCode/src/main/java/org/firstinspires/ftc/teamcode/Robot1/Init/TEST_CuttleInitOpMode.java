package org.firstinspires.ftc.teamcode.Robot1.Init;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

import org.firstinspires.ftc.teamcode.Robot1.Subsystems.Limelight;


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

    public Limelight3A camera;
    public Limelight limelight;

    @Override
    public void onInit() {
        //rev hubs
        ctrlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);

        camera = hardwareMap.get(Limelight3A.class, "limelight");

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

        limelight = new Limelight(camera);
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