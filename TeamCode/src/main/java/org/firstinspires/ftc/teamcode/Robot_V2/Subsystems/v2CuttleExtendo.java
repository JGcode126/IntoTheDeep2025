package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo.ExtendoState.FULL;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo.ExtendoState.MIDDLE;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

public class v2CuttleExtendo {

    CuttleMotor extendoMotor;
    CuttleRevHub controlHub;
    CuttleEncoder motorEncoder;
    MotorPositionController slidePosController;
    private ExtendoState currentState = INE;
    double slidePosition, slidePosOffset;

    private PIDController controller;
    private final double ticks_in_degree = 384.5/360.0;
    private static final double JOYSTICK_SCALE = 1;  // Adjust as needed make higher for less sensitive
    public static double p = 1.7, i = 0, d = 0.04;
    public static double p2 = 0.4, i2 = 0, d2 = 0;
    //0.9, 0.05

    public v2CuttleExtendo(CuttleMotor motor, CuttleEncoder encoder, MotorPositionController motorpos, CuttleRevHub hub){
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
        if (position >= 9){
            NewPosition = 9;
            //extraPower = 0.01;
        }
        if (position <= 0){
            NewPosition = 0;
            extraPower = -0.09;
        }

        controller.setPID(p, i, d);
        double pid = controller.calculate(getPos(), NewPosition);
        extendoMotor.setPower(pid + extraPower);
    }
    public void setSlidePositionColor(double position){
        //7.3 is max
        double NewPosition = position;
        double extraPower = 0;
        if (position >= 7.3){
            NewPosition = 7.3;
            //extraPower = 0.01;
        }
        if (position <= 0){
            NewPosition = 0;
            extraPower = -0.09;
        }

        controller.setPID(p2, i2, d2);
        double pid = controller.calculate(getPos(), NewPosition);
        extendoMotor.setPower(pid + extraPower);
    }

    public void hardReset(){
        // Run the motors until the current exceeds 3300 mA
        while (controlHub.getMotorCurrent(2) < 3300) {
            extendoMotor.setPower(-0.5);
        }
        // Once the loop exits (current >= 3300 mA), stop the motors and reset slides
        resetSlides();
        extendoMotor.setPower(0);
        //slidesAutoLow(0);
    }

    public void resetSlides(){
        slidePosController.setHome();
    }

    public double extendoMachine(boolean buttonIN, boolean buttonMIDDLE, boolean buttonFULLEXTEND, boolean smallExtend, boolean smallRetract){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (currentState){
            case INE:
                slidePosition = 0;
                slidePosOffset = 0;
                if(buttonMIDDLE){currentState = MIDDLE;}
                if(buttonFULLEXTEND){currentState = FULL;}
                break;
            case MIDDLE:
                slidePosition = 2.25 + slidePosOffset;
                if(smallExtend){slidePosOffset += 0.25;}
                if(smallRetract){slidePosOffset -= 0.25;}
                if(buttonIN){currentState = INE;}
                if(buttonFULLEXTEND){currentState = FULL;}
                break;
            case FULL:
                slidePosition = 5.15 + slidePosOffset;
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
