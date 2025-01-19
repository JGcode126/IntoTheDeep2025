package org.firstinspires.ftc.teamcode.Robot1.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;

@TeleOp
@Disabled
public class testColorSensor extends CuttleInitOpMode {
    private String color;
    public void onInit() {
        super.onInit();
    }
    public void mainLoop() {
        super.mainLoop();

        if(intake.colorSensor.green() > intake.colorSensor.red() && intake.colorSensor.green() > intake.colorSensor.blue() && intake.colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = "yellow";
        }
        else if(intake.colorSensor.blue() > intake.colorSensor.red() && intake.colorSensor.blue() > intake.colorSensor.green() && intake.colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = "blue";
        }
        else if(intake.colorSensor.blue() < intake.colorSensor.red() && intake.colorSensor.red() > intake.colorSensor.green() && intake.colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = "red";
        }
        telemetry.addData("color",color);
        telemetry.addData("red", intake.colorSensor.red());
        telemetry.addData("green", intake.colorSensor.green());
        telemetry.addData("blue", intake.colorSensor.blue());
        telemetry.update();
    }
}

