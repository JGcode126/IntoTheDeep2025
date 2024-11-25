package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
@Config
public class specimenAuto extends CuttleInitOpMode {
    public void onInit() {
        super.onInit();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        methods.initRobot();

        methods.test = false;
        methods.side = "right";
        methods.color = "blue";
    }

    public void main() {
        super.main();

        methods.sample();
        methods.firstSpecimen();
        methods.firstSpecimen();
        methods.secondSample();
        methods.thridSample();

        methods.scoringSpecimen();
        methods.scoringSpecimen();
        methods.scoringSpecimen();

        methods.teleOpInit();
    }

    public void mainLoop() {
        super.mainLoop();
        methods.telemetryData();
    }

    private enum State {
        TO_SAMPLE,
        INTAKING_SAMPLE,
        SPIT_OUT_SAMPLE
    }
}
