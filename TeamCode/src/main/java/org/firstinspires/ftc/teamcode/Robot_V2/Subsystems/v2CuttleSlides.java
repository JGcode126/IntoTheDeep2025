package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides.LiftState.HIGHSUB;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides.LiftState.HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides.LiftState.IN;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides.LiftState.LOWSUB;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides.LiftState.LOW_BUCKET;

import com.arcrobotics.ftclib.controller.PIDController;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public class v2CuttleSlides {

    public v2CuttleSlides.LiftState currentState = IN;
    CuttleMotor liftMotorLeft;
    CuttleMotor liftMotorRight;
    CuttleRevHub controlHub;
    CuttleEncoder liftmotorEncoder;
    MotorPositionController liftPosController;
    //private LiftState currentState = IN;
    double liftPosition, positionOffset = 0, positionOffset2 = 0;

    private PIDController controller;
    public static double p = 0.4, i = 0.0, d = 0;
    private double alpha = 0.775;
    private double power = 0;
    private double filteredPosition = 0.0; // Initial filtered position


    public v2CuttleSlides(CuttleMotor motorleft, CuttleMotor motorright, CuttleEncoder encoder, MotorPositionController motorpos, CuttleRevHub hub){
        liftMotorLeft = motorleft;
        liftMotorRight = motorright;
        controlHub = hub;
        liftPosController = motorpos;
        liftmotorEncoder = encoder;
        controller = new PIDController(p, i, d);
    }

    public double getPos(){
        return liftPosController.getHomedEncoderPosition();
    }


    public void setLiftPosition(double position) {

        double NewPosition = position;
        double ff = 0.16;
        if (position >= 10.2){
            NewPosition = 10.2;
        }
        if (position <= 0){
            NewPosition = -0.01;
        }

        controller.setPID(p, i, d);
        double pid = controller.calculate(getPos(), NewPosition);
        double pidDirection = pid;
        if (-pidDirection > 0){
            ff = ff * -1;
        }

        double power = (pid + ff) * -1;

        if (power > 0) { // Only when moving down
            power = power * 0.04;
        }

        if (getPos() < 0.1 && power > -0.3){
            power = 0.05;
        }

        liftMotorRight.setPower(power);
        liftMotorLeft.setPower(power);



        /*
        // Clamp the target position within the allowed range
        double clampedTarget = Math.max(-0.01, Math.min(10.2, targetPosition));

        // Low-pass filtering on the current position
        double currentPos = getPos();
        filteredPosition = alpha * currentPos + (1 - alpha) * filteredPosition;

        // Calculate the distance to the target
        double error = clampedTarget - filteredPosition;

        // PID control using the filtered position
        controller.setPID(p, i, d);
        double pid = controller.calculate(filteredPosition, clampedTarget);

        // Feedforward term
        double ff = 0.08;

        // Calculate base power
        double power = (pid + ff) * -1;

        // Apply scaling only when the slides are moving down
        if (power > 0) { // Only when moving down
            power = power * 0.04;
        }

        // Set the motor power
        liftMotorRight.setPower(power);
        liftMotorLeft.setPower(power);

         */
    }

    public void setLiftPositionFaster(double position) {
        double NewPosition = position;
        double ff = 0.16;
        if (position >= 10.2){
            NewPosition = 10.2;
        }
        if (position <= 0){
            NewPosition = -0.01;
        }

        controller.setPID(p, i, d);
        double pid = controller.calculate(getPos(), NewPosition);
        double pidDirection = pid;
        if (-pidDirection > 0){
            ff = ff * -1;
        }

        double power = (pid + ff) * -1;

        if (getPos() < 0.1 && power > -0.3){
            power = 0.05;
        }

        liftMotorRight.setPower(power);
        liftMotorLeft.setPower(power);
        /*
        // Clamp the target position within the allowed range
        double clampedTarget = Math.max(-0.01, Math.min(10.2, targetPosition));

        // Low-pass filtering on the current position
        double currentPos = getPos();
        filteredPosition = alpha * currentPos + (1 - alpha) * filteredPosition;

        // Calculate the distance to the target
        double error = clampedTarget - filteredPosition;

        // PID control using the filtered position
        controller.setPID(p, i, d);
        double pid = controller.calculate(filteredPosition, clampedTarget);

        // Feedforward term
        double ff = 0.08;

        // Calculate base power
        double power = (pid + ff) * -1;

        // Apply scaling only when the slides are moving down
        if (error < 0) { // Only when moving down
            double scalingFactor = Math.max(0.1, Math.min(1.0, Math.abs(error) / 10.0));
            power *= scalingFactor;
        }

        // Set the motor power
        liftMotorRight.setPower(power);
        liftMotorLeft.setPower(power);

         */
    }


    public double liftMachine(boolean buttonIN, boolean buttonLOWBUCKET, boolean buttonHIGHBUCKET, boolean buttonLOWSUB, boolean buttonHIGHSUB, boolean upOffset, boolean downOffset){
        switch (currentState){
            case IN:
                liftPosition = 0; //good
                positionOffset = 0;
                if(buttonLOWBUCKET){currentState = LOW_BUCKET;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                if(buttonHIGHSUB){currentState = HIGHSUB;}
                break;
            case LOW_BUCKET:
                liftPosition = 4.5; //good
                if(buttonIN){currentState = IN;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                if(buttonHIGHSUB){currentState = HIGHSUB;}
                break;
            case HIGH_BUCKET:
                liftPosition = 10.2; //good
                if(buttonIN){currentState = IN;}
                if(buttonLOWBUCKET){currentState = LOW_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                if(buttonHIGHSUB){currentState = HIGHSUB;}
                break;
            case LOWSUB:
                liftPosition = 0.5 +positionOffset; //kinda impossible...
                if(upOffset){positionOffset += 0.25;}
                if(downOffset){positionOffset -= 0.25;}
                if(buttonIN){currentState = IN;}
                if(buttonLOWBUCKET){currentState = LOW_BUCKET;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonHIGHSUB){currentState = HIGHSUB;}
                break;
            case HIGHSUB:
                liftPosition = 4.5 + positionOffset;
                if(upOffset){positionOffset += 0.2;}
                if(downOffset){positionOffset -= 0.25;}
                if(buttonIN){currentState = IN;}
                if(buttonLOWBUCKET){currentState = LOW_BUCKET;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                break;
            case BACKINTAKEPOS:
                //2
                liftPosition = 2.95 + positionOffset;
                if(upOffset){positionOffset += 0.05;}
                if(downOffset){positionOffset -= 0.05;}
                if(buttonIN){currentState = IN;}
                if(buttonLOWBUCKET){currentState = LOW_BUCKET;}
                if(buttonHIGHSUB){currentState = HIGHSUB;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                break;
            case FRONTSCOREPOS:
                liftPosition = 3.7 + positionOffset2;
                if(upOffset){positionOffset2 += 0.2;}
                if(downOffset){positionOffset2 -= 0.25;}
                if(buttonIN){currentState = IN;}
                if(buttonLOWBUCKET){currentState = LOW_BUCKET;}
                if(buttonHIGHSUB){currentState = HIGHSUB;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                break;

        }
        return liftPosition;
    }

    public enum LiftState {
        IN, HIGH_BUCKET, LOW_BUCKET, HIGHSUB, LOWSUB, BACKINTAKEPOS, FRONTSCOREPOS
    }

    public void setLiftState(LiftState state){
        currentState = state;
    }

    public void hardReset(){
        liftMotorRight.setPower(-0.5);
        liftMotorLeft.setPower(0.5);
    }

}
