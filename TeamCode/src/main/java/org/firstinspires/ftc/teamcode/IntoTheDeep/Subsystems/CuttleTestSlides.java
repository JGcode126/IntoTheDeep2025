package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Utilities.PDFL;

public class CuttleTestSlides {
    CuttleMotor slideMotor;
    CuttleRevHub expHub;
    CuttleEncoder motorEncoder;
    MotorPositionController slidePosController;

    private PDFL controller;
    private final double ticks_in_degree = 384.5/360.0;
    private static final double JOYSTICK_SCALE = 1;  // Adjust as needed make higher for less sensitive
    public static double p = 0.6, d = 0.008, f = -0.06, l = 0.00;
    public static int target = 0;

    public CuttleTestSlides(CuttleMotor motor, CuttleEncoder encoder, MotorPositionController motorpos, CuttleRevHub expHub){
        slideMotor = motor;
        motorEncoder = encoder;
        slidePosController = motorpos;
        expHub = expHub;

        controller = new PDFL(p,d,f,l);
    }

    public void moveSlides(double left_stick_y){
        double slidePower = -left_stick_y / JOYSTICK_SCALE; // Reverse the sign if needed

        // Update target position based on joystick input
        target += (int) (slidePower / JOYSTICK_SCALE);  // Adjust multiplier as needed

        // Ensure the target position is not below x
        //target = Math.max(target, 0);

        double armPos = slidePosController.getHomedEncoderPosition();
        double error = armPos-target;
        double power = controller.run((int)Math.round(error));

        slideMotor.setPower(power*0.3);
    }

    public void slidesLow(){
        target = 1;

        double armPos = slidePosController.getHomedEncoderPosition();
        double error = armPos-target;
        double power = controller.run((int)Math.round(error));

        slideMotor.setPower(power*0.6);
    }

    public double getPos(){
        return slidePosController.getHomedEncoderPosition();
    }

    public void resetSlides(){
        slidePosController.setHome();
    }
}
