package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public class CuttleDT{
    public CuttleMotor leftFrontMotor;
    public CuttleMotor rightFrontMotor;
    public CuttleMotor rightBackMotor;
    public CuttleMotor leftBackMotor;

    CuttleRevHub expansion;
    CuttleRevHub control;

    double last_error = 0;
    double integral = 0;
    double releaseAngle;
    double adjRateOfChange;
    double targetAngle;
    double inputTurn;

    public CuttleDT(CuttleMotor motor1, CuttleMotor motor2, CuttleMotor motor3, CuttleMotor motor4, CuttleRevHub ehub, CuttleRevHub chub){
        leftBackMotor = motor1;
        leftFrontMotor = motor2;
        rightBackMotor = motor3;
        rightFrontMotor = motor4;
        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);
        expansion = ehub;
        control = chub;

        leftBackMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double drive, double strafe, double turn){

        leftFrontMotor.setPower((drive+strafe+turn)*-1);
        rightFrontMotor.setPower((drive-strafe-turn)*-1);
        leftBackMotor.setPower((drive-strafe+turn)*-1);
        rightBackMotor.setPower((drive+strafe-turn)*-1);
    }
    public void stop(){
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}

