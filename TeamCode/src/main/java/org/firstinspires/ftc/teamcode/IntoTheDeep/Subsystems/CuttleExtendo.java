package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

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

    private PIDController controller;
    private final double ticks_in_degree = 384.5/360.0;
    private static final double JOYSTICK_SCALE = 1;  // Adjust as needed make higher for less sensitive
    public static double p = 3.56, i = 0.0, d = 0.01;
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
    public boolean isInRange(double target){
        boolean answer = false;
        answer= extendoMotor.power < 0.1; //getPos() > target - 0.3 && getPos() < target + 0.3;
        return answer;
    }

    public void setSlidePosition(double position){
        //7.3 is max

        double NewPosition = position;
        if (position > 7.3){
            NewPosition = 7.3;
        }
        controller.setPID(p, i, d);
        double pid = controller.calculate(getPos(), NewPosition);

        extendoMotor.setPower(pid-0.1);

    }





}
