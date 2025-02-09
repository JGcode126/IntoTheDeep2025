package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class TimeBasedCuttleHang {
    public CRServo Rhang, Lhang;

    public TimeBasedCuttleHang(HardwareMap hardwareMap){
        Lhang = hardwareMap.get(CRServo.class, "hang left");
        Rhang = hardwareMap.get(CRServo.class, "hang right");
    }

    public void hangUp(double time){
        if (time < 3.8) {
            Lhang.setPower(1);
            Rhang.setPower(-1);
        } else {
            Lhang.setPower(0);
            Rhang.setPower(0);
        }
    }

    public void hangUpAuto(double time){
        if (time < 1) {
            Lhang.setPower(1);
            Rhang.setPower(-1);
        } else {
            Lhang.setPower(0);
            Rhang.setPower(0);
        }
    }

    public void hangDown(double time){
        if (time < 6) {
            Lhang.setPower(-1);
            Rhang.setPower(1);
        } else {
            Lhang.setPower(0);
            Rhang.setPower(0);
        }
    }
}
