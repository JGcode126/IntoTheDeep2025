package org.firstinspires.ftc.teamcode.Robot_V2.Autos.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboctopi.cuttlefish.queue.TaskList;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;

@Autonomous
@Disabled
@Config
public class intake extends CuttleInitOpModeRobot2 {

    public void onInit(){
        super.onInit();
    }

    public void main(){
        super.main();
        intake.in();

    }

    public void mainLoop() {
        super.mainLoop();
    }
}
