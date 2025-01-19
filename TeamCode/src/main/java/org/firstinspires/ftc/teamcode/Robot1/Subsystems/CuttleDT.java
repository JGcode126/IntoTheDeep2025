package org.firstinspires.ftc.teamcode.Robot1.Subsystems;

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

    public void driveDO(double drive, double strafe, double turn, double fast, double superslow, double rotation){

        double[] rotatedVector = rotateVector(strafe, drive, rotation);

        // Update strafe and drive with the rotated components
        drive = rotatedVector[1];
        strafe = rotatedVector[0];

        if(turn!= 0) {
            inputTurn = turn;
            releaseAngle = Math.toDegrees(rotation);
        } else{
            targetAngle = releaseAngle;
            inputTurn = PID(targetAngle-Math.toDegrees(rotation), 0.00,0,0.00);
        }

        if (fast > 0.02) {
            leftFrontMotor.setPower((drive+strafe+(inputTurn))*-0.6);
            rightFrontMotor.setPower((drive-strafe-(inputTurn))*-0.6);
            leftBackMotor.setPower((drive-strafe+(inputTurn))*-0.6);
            rightBackMotor.setPower((drive+strafe-(inputTurn))*-0.6);
        } else if (superslow > 0.02) {
            leftFrontMotor.setPower((drive+strafe*1.5+inputTurn)*-0.25);
            rightFrontMotor.setPower((drive-strafe*1.5-inputTurn)*-0.25);
            leftBackMotor  .setPower((drive-strafe*1.5+inputTurn)*-0.25);
            rightBackMotor.setPower((drive+strafe*1.5-inputTurn)*-0.25);
        } else{
            leftFrontMotor.setPower((drive+strafe+inputTurn*0.9)*-1);
            rightFrontMotor.setPower((drive-strafe-inputTurn*0.9)*-1);
            leftBackMotor.setPower((drive-strafe+inputTurn*0.9)*-1);
            rightBackMotor.setPower((drive+strafe-inputTurn*0.9)*-1);
        }

    }


    public void driveNonDO(double drive, double strafe, double turn, double fast, double superslow, double rotation){

        if(turn!= 0) {
            inputTurn = turn;
            releaseAngle = Math.toDegrees(rotation);
        } else{
            targetAngle = releaseAngle;
            inputTurn = PID(targetAngle-Math.toDegrees(rotation), 0.00,0,0.00);
        }

        if (fast > 0.02) {
            leftFrontMotor.setPower((drive+strafe+(inputTurn))*-0.6);
            rightFrontMotor.setPower((drive-strafe-(inputTurn))*-0.6);
            leftBackMotor.setPower((drive-strafe+(inputTurn))*-0.6);
            rightBackMotor.setPower((drive+strafe-(inputTurn))*-0.6);
        } else if (superslow > 0.02) {
            leftFrontMotor.setPower((drive+strafe*1.5+inputTurn)*-0.25);
            rightFrontMotor.setPower((drive-strafe*1.5-inputTurn)*-0.25);
            leftBackMotor  .setPower((drive-strafe*1.5+inputTurn)*-0.25);
            rightBackMotor.setPower((drive+strafe*1.5-inputTurn)*-0.25);
        } else{
            leftFrontMotor.setPower((drive+strafe+inputTurn*0.9)*-1);
            rightFrontMotor.setPower((drive-strafe-inputTurn*0.9)*-1);
            leftBackMotor.setPower((drive-strafe+inputTurn*0.9)*-1);
            rightBackMotor.setPower((drive+strafe-inputTurn*0.9)*-1);
        }

    }


    public double PID (double error, double kp, double ki, double kd){

        integral += error;
        double derivative = error - last_error;
        double preportional = error * kp;
        double integral2 = integral * ki;
        double derivative2 = derivative * kd;

        double correction = preportional + integral2 + derivative2;
        return correction;

    }

    public void stop(){
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public static double[] rotateVector(double x, double y, double radians) {
        double newX = x * Math.cos(radians) - y * Math.sin(radians);
        double newY = x * Math.sin(radians) + y * Math.cos(radians);
        return new double[]{newX, newY};
    }


}

