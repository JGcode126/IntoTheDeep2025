package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class v2CuttleSweep {
    public CuttleServo broom;

    public v2CuttleSweep(CuttleServo servo1){
        broom = servo1;
    }

    public void broomIn(){
        broom.setPosition(0.15);
    }

    public void broomOut(){
        broom.setPosition(0.65);
    }
    public void broomStraight(){
        broom.setPosition(0.5);
    }


}
