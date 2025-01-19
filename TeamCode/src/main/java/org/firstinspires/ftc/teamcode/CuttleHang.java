package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CuttleHang {
    public CRServo hang1;
    public CuttleHang(HardwareMap hardwareMap){
        hang1 = hardwareMap.get(CRServo.class,"hang1");
    }

    public void servoUp(){
        hang1.setPower(1);
    }
}
