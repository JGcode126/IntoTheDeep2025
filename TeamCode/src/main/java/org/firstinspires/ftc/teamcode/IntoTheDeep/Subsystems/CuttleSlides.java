package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.HIGHSUB;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.HIGH_BUCKET;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.IN;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.LOWSUB;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides.LiftState.LOW_BUCKET;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public class CuttleSlides {

    public CuttleSlides.LiftState currentState = IN;
    CuttleMotor liftMotorLeft;
    CuttleMotor liftMotorRight;
    CuttleRevHub controlHub;
    CuttleEncoder liftmotorEncoder;
    MotorPositionController liftPosController;
    //private LiftState currentState = IN;
    double liftPosition, positionOffset = 0;

    private PIDController controller;
    public static double p = 1, i = 0.0, d = 0.04;


    public CuttleSlides(CuttleMotor motorleft, CuttleMotor motorright, CuttleEncoder encoder, MotorPositionController motorpos, CuttleRevHub hub){
        liftMotorLeft = motorleft;
        liftMotorRight = motorright;
        controlHub = hub;
        liftPosController = motorpos;
        liftmotorEncoder = encoder;
        liftMotorLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
    }

    public double getPos(){
        return liftPosController.getHomedEncoderPosition();
    }
    public void setLiftPosition(double position){
        //7.3 is max
        double NewPosition = position;
        double ff = 0.08;
        if (position >= 10.4){
            NewPosition = 11;
        }
        if (position <= 0){
            NewPosition = 0;
        }

        controller.setPID(p, i, d);
        double pid = controller.calculate(getPos(), NewPosition);
        if (liftMotorRight.power < 0){
            liftMotorLeft.setPower((pid+ ff) * -0.1);
            liftMotorRight.setPower((pid + ff) * 0.1);
        } else {
            liftMotorLeft.setPower((pid + ff) * -1);
            liftMotorRight.setPower((pid + ff) * 1);
        }
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
                liftPosition = 6; //good
                if(buttonIN){currentState = IN;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                if(buttonHIGHSUB){currentState = HIGHSUB;}
                break;
            case HIGH_BUCKET:
                liftPosition = 13; //good
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
                liftPosition = 5.1 + positionOffset;
                if(upOffset){positionOffset += 0.4;}
                if(downOffset){positionOffset -= 0.25;}
                if(buttonIN){currentState = IN;}
                if(buttonLOWBUCKET){currentState = LOW_BUCKET;}
                if(buttonHIGHBUCKET){currentState = HIGH_BUCKET;}
                if(buttonLOWSUB){currentState = LOWSUB;}
                break;
        }
        return liftPosition;
    }

    public enum LiftState {
        IN, HIGH_BUCKET, LOW_BUCKET, HIGHSUB, LOWSUB
    }

    public void setLiftState(LiftState state){
        currentState = state;
    }

    public void hardReset(){
        liftMotorRight.setPower(-0.5);
        liftMotorLeft.setPower(0.5);
    }

}
