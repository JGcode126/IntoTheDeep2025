package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class v2CuttleHang {
    public CuttleServo hangR, hangL;

    /*public CuttleHang(HardwareMap hardwareMap){
        hangR = hardwareMap.get(CRServo.class,"hangR");
        hangL = hardwareMap.get(CRServo.class, "hangL");
    }*/

    public v2CuttleHang(CuttleServo servo1, CuttleServo servo2){
        hangR = servo1;
        hangL = servo2;
    }

    public void initHang(){
        hangL.setPosition(0.5);
        hangR.setPosition(0.5);
    }

    public void teleHeight(){
        hangL.setPosition(0.79);
        hangR.setPosition(0.2);
    }

    public void hangHeight(){
        hangL.setPosition(0.55);
        hangR.setPosition(0.45);
    }

    public void setHeight(double hL, double hR){
        hangR.setPosition(hR);
        hangL.setPosition(hL);
    }
}
