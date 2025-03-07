package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class v2CuttleSweep {
    public Servo broom;

    public v2CuttleSweep(CuttleServo servo1, HardwareMap hardwareMap){
        broom = hardwareMap.get(Servo.class, "broom");
    }

    public void broomIn(){
        broom.setPosition(0.13);
    }

    public void broomOut(){
        broom.setPosition(0.65);
    }
    public void broomStraight(){
        broom.setPosition(0.5);
    }

    public void broomSet(double amount){
        broom.setPosition(amount);
    }



}
