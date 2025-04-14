package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class StraferChassisTele extends LinearOpMode {
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;
    @Override
    public void runOpMode(){
        motor1 = hardwareMap.get(DcMotor.class, "motor1");//top left
        motor2 = hardwareMap.get(DcMotor.class, "motor2");//top right
        motor3 = hardwareMap.get(DcMotor.class, "motor3");//bottom left
        motor4 = hardwareMap.get(DcMotor.class, "motor4");//bottom right

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;


        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motor1.setPower(frontLeftPower/2);
            motor3.setPower(backLeftPower/2);
            motor2.setPower(frontRightPower/2);
            motor4.setPower(backRightPower/2 );
        }
    }
}
