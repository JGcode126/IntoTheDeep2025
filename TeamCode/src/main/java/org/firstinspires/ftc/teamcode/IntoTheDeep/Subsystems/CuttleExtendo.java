package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo.State.DOWN;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Utilities.PDFL;

public class CuttleExtendo {
    CuttleMotor extendoMotor;
    CuttleRevHub controlHub;
    CuttleEncoder motorEncoder;
    MotorPositionController slidePosController;
    private CuttleExtendo.State currentState = DOWN;

    private PIDController controller;
    private final double ticks_in_degree = 384.5/360.0;
    private static final double JOYSTICK_SCALE = 1;  // Adjust as needed make higher for less sensitive
    public static double p = 3, i = 0.0, d = 0.01;
    public static double f = -0.06;
    public static int target = 0;

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

    public void resetSlides(){
        slidePosController.setHome();
    }


    public void setSlidePosition(double position){
        //7.3 is max
        double NewPosition = position;
        if (position > 7.3){
            NewPosition = 7.3;
        }
        if (position < 0){
            NewPosition = 0;
        }

        controller.setPID(p, i, d);
        double pid = controller.calculate(getPos(), NewPosition);
        extendoMotor.setPower(pid);
    }

    public void liftMachine(double buttona, double buttonb, boolean buttonc, boolean buttond, boolean buttone){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (currentState){
            case DOWN:
                setSlidePosition(0);
                break;
            case MIDDLE:
                setSlidePosition(3.5);
                break;
            case UP:
                setSlidePosition(7.3);
                break;
        }
    }

    public enum State {
        DOWN, MIDDLE, UP
    }




}
