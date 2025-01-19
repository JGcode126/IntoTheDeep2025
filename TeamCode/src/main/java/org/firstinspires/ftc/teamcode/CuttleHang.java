package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class CuttleHang {
    public CuttleServo hangR, hangL;

    /*public CuttleHang(HardwareMap hardwareMap){
        hangR = hardwareMap.get(CRServo.class,"hangR");
        hangL = hardwareMap.get(CRServo.class, "hangL");
    }*/

    public CuttleHang(CuttleServo servo1, CuttleServo servo2){
        hangR = servo1;
        hangL = servo2;
    }

    public void initHang(){
        hangL.setPosition(0);
        hangR.setPosition(-0.2);
    }

    public void servoUp(){
        hangR.setPosition(-1);
        hangL.setPosition(-1);
    }
    public void servoDown(){
        hangR.setPosition(1);
        hangL.setPosition(1);
    }

    public void setHeight(double hL, double hR){
        hangR.setPosition(hR);
        hangL.setPosition(hL);
    }
}
