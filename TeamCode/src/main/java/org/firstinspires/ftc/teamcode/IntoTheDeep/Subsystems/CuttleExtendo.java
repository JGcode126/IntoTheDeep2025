package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.FULL;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.ExtendoState.MIDDLE;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public class CuttleExtendo {

    CuttleMotor extendoMotor;
    CuttleRevHub controlHub;
    CuttleEncoder motorEncoder;
    MotorPositionController slidePosController;
    private ExtendoState currentState = INE;
    double slidePosition, slidePosOffset;

    private PIDController controller;
    private final double ticks_in_degree = 384.5/360.0;
    private static final double JOYSTICK_SCALE = 1;  // Adjust as needed make higher for less sensitive
    public static double p = 1.9, i = 0.0, d = 0.01;


    public CuttleExtendo(CuttleMotor motor, CuttleEncoder encoder, MotorPositionController motorpos, CuttleRevHub hub){
        extendoMotor = motor;
        controlHub = hub;
        slidePosController = motorpos;
        motorEncoder = encoder;
        extendoMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
    }

    public double getPos(){
        return slidePosController.getHomedEncoderPosition();
    }

    public void setSlidePosition(double position){
        //7.3 is max
        double NewPosition = position;
        double extraPower = 0;
        if (position >= 7.3){
            NewPosition = 7.3;
            extraPower = 0.01;
        }
        if (position <= 0){
            NewPosition = 0;
            extraPower = -0.09;
        }

        controller.setPID(p, i, d);
        double pid = controller.calculate(getPos(), NewPosition);
        extendoMotor.setPower(pid + extraPower);
    }

    public void hardRetract(){
        //work in progress
        extendoMotor.setPower(-0.5);
    }

    public double extendoMachine(boolean buttonIN, boolean buttonMIDDLE, boolean buttonFULLEXTEND, boolean smallExtend, boolean smallRetract){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (currentState){
            case INE:
                slidePosition = 0 + slidePosOffset;
                if(smallExtend){slidePosOffset += 0.25;}
                if(smallRetract){slidePosOffset -= 0.25;}
                if(buttonMIDDLE){currentState = MIDDLE;}
                if(buttonFULLEXTEND){currentState = FULL;}
                break;
            case MIDDLE:
                slidePosition = 3 + slidePosOffset;
                if(smallExtend){slidePosOffset += 0.25;}
                if(smallRetract){slidePosOffset -= 0.25;}
                if(buttonIN){currentState = INE;}
                if(buttonFULLEXTEND){currentState = FULL;}
                break;
            case FULL:
                slidePosition = 6 + slidePosOffset;
                if(smallRetract){slidePosOffset -= 0.25;}
                if(buttonIN){currentState = INE;}
                if(buttonMIDDLE){currentState = MIDDLE;}
                break;

        }
        return slidePosition;
    }

    public enum ExtendoState {
        INE, MIDDLE, FULL
    }

    public void setExtendoState(ExtendoState state){
        currentState = state;
    }




}
