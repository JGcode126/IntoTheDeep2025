package org.firstinspires.ftc.teamcode.Robot1.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot1.Init.TEST_CuttleInitOpMode;

@TeleOp
@Disabled
public class sampleDetection extends TEST_CuttleInitOpMode {
    public void onInit() {
        super.onInit();
        limelight.initCam(0);
    }

    public void main() {}

    public void mainLoop() {
        super.mainLoop();
        limelight.detect();
    }
}
