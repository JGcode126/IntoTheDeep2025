package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class v2CuttleHang {
    public CuttleServo hangR, hangL;

    public v2CuttleHang(CuttleServo servo1, CuttleServo servo2){
        hangL = servo1;
        hangR = servo2;
    }

    public void initHang(){
        hangR.setPosition(0.5);
        hangL.setPosition(0.5);
    }

    public void teleHeight(){
        hangR.setPosition(0.9);
        hangL.setPosition(0.08);
    }

    public void hangHeight(){
        hangR.setPosition(0.15);
        hangL.setPosition(0.85);
    }

    public void parkHeight(){
        hangR.setPosition(0.65);
        hangL.setPosition(0.35);
    }

    public void setHeight(double hL, double hR){
        hangR.setPosition(hR);
        hangL.setPosition(hL);
    }
}
