package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

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

    //private PDFL controller;
    private final double ticks_in_degree = 384.5/360.0;
    private static final double JOYSTICK_SCALE = 1;  // Adjust as needed make higher for less sensitive
    public static double p = 0.6, i = 0.0, d = 0.008;
    public static double f = -0.06;
    public static int target = 0;

    public CuttleExtendo(CuttleMotor motor, CuttleEncoder encoder, MotorPositionController motorpos, CuttleRevHub hub){
        extendoMotor = motor;
        controlHub = hub;
        slidePosController = motorpos;
        motorEncoder = encoder;

        //controller = new PIDController(p, i, d);
    }

    public double getPos(){
        return slidePosController.getHomedEncoderPosition();
    }



    public void resetSlides(){
        slidePosController.setHome();
    }
}
