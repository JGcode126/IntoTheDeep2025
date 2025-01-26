package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.IntakeState.TRANSFERED;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake.OutakeState.BARLEFT;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake.OutakeState.BARRIGHT;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake.OutakeState.READY;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

@TeleOp
@Config
public class Tester_opmode extends CuttleInitOpModeRobot2{
    public void onInit() {
        super.onInit();

    }
    public void main() {
        super.main();
    }
    public void mainLoop() {
        super.mainLoop();
        outake.readyPos();

        telemetry.addData("Color:", intake.getColor());
        telemetry.update();
    }





}
