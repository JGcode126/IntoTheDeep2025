package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.State.FULL;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.State.IN;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.State.MIDDLE;

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
    private CuttleExtendo.State currentState = IN;
    double slidePosition;

    private PIDController controller;
    private final double ticks_in_degree = 384.5/360.0;
    private static final double JOYSTICK_SCALE = 1;  // Adjust as needed make higher for less sensitive
    public static double p = 2, i = 0.0, d = 0.04;


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

    public double extendoMachine(boolean buttona, boolean buttonb, boolean buttonc){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (currentState){
            case IN:
                slidePosition = 0;
                if(buttonb){currentState = MIDDLE;}
                if(buttonc){currentState = FULL;}
                break;
            case MIDDLE:
                slidePosition = 4;
                if(buttona){currentState = IN;}
                if(buttonc){currentState = FULL;}
                break;
            case FULL:
                slidePosition = 7.5;
                if(buttona){currentState = IN;}
                if(buttonb){currentState = MIDDLE;}
                break;

        }
        return slidePosition;
    }

    public enum State {
        IN, MIDDLE, FULL
    }




}
