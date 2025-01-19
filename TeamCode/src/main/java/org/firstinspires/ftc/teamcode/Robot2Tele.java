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
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;

@TeleOp
@Config
public class Robot2Tele extends CuttleInitOpModeRobot2{
    double hL = 0;
    double hR = 0;

    public void onInit() {
        super.onInit();
        hang.initHang();
    }
    public void main() {
        super.main();
        //hang.servoUp();
    }
    public void mainLoop() {
        super.mainLoop();

        if(gamepad1.left_bumper){
            if (hL < 1.0){
                hL += 0.01;
            }
        }
        if(gamepad1.right_bumper){
            if(hL > -1.0) {
                hL -= 0.01;
            }
        }

        if(gamepad2.left_bumper){
            if (hR < 1.0){
                hR += 0.01;
            }
        }
        if(gamepad2.right_bumper){
            if(hR > -1.0) {
                hR -= 0.01;
            }
        }

        hang.setHeight(hL,hR);
        telemetry.addData("hl:",hL);
        telemetry.addData("hr", hR);
        telemetry.update();
    }

}
