package org.firstinspires.ftc.teamcode.Robot1.RegularlyUsed;

public class BatteryOLD {
    //double new Power = powerToBeSentToMotor / batteryVoltage * 13
    int batteryV;
    double optimalV;

    public BatteryOLD(int voltage, double optimalVoltage){
        this.batteryV = voltage;
        this.optimalV = optimalVoltage;
    }
    double optimalSpeed(double batteryVoltage, double optimalVoltage, double optimalSpeed){
        return optimalVoltage/batteryVoltage * optimalSpeed;
    }

    double optimalSpeed(double speed){
        return optimalV/batteryV * speed;
    }
}
