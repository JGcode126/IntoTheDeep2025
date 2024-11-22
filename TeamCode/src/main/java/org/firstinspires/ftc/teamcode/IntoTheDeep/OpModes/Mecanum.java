package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;


@TeleOp
//@Disabled
public class Mecanum extends CuttleInitOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public void onInit() {super.onInit(); timer.reset();}
    public void main() {
        super.main();
    }
    public void mainLoop() {
        super.mainLoop();
        dt.drive;
        dt.drive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        if (gamepad .1 right_stick_y > 1 || gamepad .1 right_stick_y < 1){
            dt.drive;
        }
    }


    /*
    // code from 10/10/24 - robot drives forwards for 3 seconds
    private ElapsedTime timer = new ElapsedTime();

    public void onInit() {super.onInit(); timer.reset();}
    public void main() {
        super.main();
    }
    public void mainLoop() {
        super.mainLoop();

        if(timer.seconds() >= 3) {
            dt.stop();
        }

        else {
            dt.drive(1, 0, 0);
        }

        telemetry.addData("seconds",timer.seconds());
         telemetry. update();
     */
    //    drivetrain.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x,gamepad1.left_stick_x, gamepad1.right_trigger);

    }
}