package org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.TRANSFERED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;

@TeleOp
@Config
public class Robot1Tele extends CuttleInitOpMode{
    double finalSlidePos = 0;
    public static double slidepos = 0;
    public void onInit() {
        super.onInit();

        intake.initPos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void main() {
        super.main();
        liftPosController.setHome();
    }

    public void mainLoop() {
        super.mainLoop();

        if (intake.intakeState == TRANSFERED){
            finalSlidePos = extendo.extendoMachine(true, false, false);
        } else{
            finalSlidePos = extendo.extendoMachine(gamepad1.a, gamepad1.x, gamepad1.y);
        }

        extendoPosition = finalSlidePos;

        intake.intakeMachine(gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left);
        dt.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        //liftPosition = lift.liftMachine(gamepad2.a, gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.b, gamepad2.x);

        outake.outakeMachine(gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.dpad_up, gamepad2.dpad_left, gamepad2.dpad_right);

        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("extendo", extendoPosController.getPosition());
        telemetry.addData("lift", liftPosController.getPosition());
        telemetry.addData("color", intake.getColor());
        telemetry.addData("intake state", intake.intakeState);
        telemetry.update();
    }


}
