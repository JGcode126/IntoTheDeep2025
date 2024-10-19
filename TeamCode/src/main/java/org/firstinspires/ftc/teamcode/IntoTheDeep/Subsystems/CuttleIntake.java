package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class CuttleIntake {
    public CRServo intakeMotor;
    CuttleServo leftServo, rightServo, turntable, clawServo;

    public CuttleIntake(CuttleServo left, CuttleServo right, CuttleServo claw, CuttleServo tt, HardwareMap hardwareMap){
        leftServo = left;
        rightServo = right;
        clawServo = claw;
        turntable = tt;
        intakeMotor = hardwareMap.get(CRServo.class,"intake");
    }

    public void initPos(){
        leftServo.setPosition(0.965);
        rightServo.setPosition(0.035);
        turntable.setPosition(0.5);
        intakeMotor.setPower(0);
    }

    public void intakePos(double turntableAngle){
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        turntable.setPosition(0.5);
    }

    public void intakeMotorOn(){
        intakeMotor.setPower(-1);
    }

    public void intakeMotorOff(){
        intakeMotor.setPower(0);
    }


}
