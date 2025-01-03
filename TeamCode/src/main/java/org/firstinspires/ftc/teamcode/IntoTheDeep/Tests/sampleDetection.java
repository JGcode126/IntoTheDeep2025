package org.firstinspires.ftc.teamcode.IntoTheDeep.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.TEST_CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;

@TeleOp
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
