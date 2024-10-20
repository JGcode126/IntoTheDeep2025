package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.REJECT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.DOWN;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.LOOKING;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.SECURED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.TRANSFERED;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes.Robot1Tele;

public class CuttleIntake{
    public CRServo intakeMotor;
    public ColorRangeSensor colorSensor;
    CuttleServo leftServo, rightServo, turntable, clawServo;
    public CuttleIntake.IntakeState intakeState = UP;


    public CuttleIntake(CuttleServo left, CuttleServo right, CuttleServo claw, CuttleServo tt, HardwareMap hardwareMap){
        leftServo = left;
        rightServo = right;
        clawServo = claw;
        turntable = tt;
        intakeMotor = hardwareMap.get(CRServo.class,"intake");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");
    }


    public void initPos(){
        leftServo.setPosition(0.965);
        rightServo.setPosition(0.035);
        turntable.setPosition(0.5);
        intakeMotor.setPower(0);
        clawServo.setPosition(0.8);
    }

    public void armUp(){
        leftServo.setPosition(0.965);
        rightServo.setPosition(0.035);
        turntable.setPosition(0.5);
        intakeMotor.setPower(0);
    }
    //close claw pos 0.957
    public void intakePos(double turntableAngle){
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        turntable.setPosition(turntableAngle);
        clawServo.setPosition(0.8);
    }

    public void nomnom (){
        if (getColor() == YELLOW || getColor() == RED || getColor() == BLUE){
            clawServo.setPosition(0.957);
            intakeMotor.setPower(0);
        }
    }

    public Color getColor(){
        Color color = null;
        colorSensor.enableLed(false);
        if (colorSensor.green() > 2100 && colorSensor.red() < 2100 && colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = YELLOW;
        } else if (colorSensor.blue() < colorSensor.red() && colorSensor.red() > colorSensor.green() && colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = RED;
        } else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green() && colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = BLUE;
        } else{
            color = null;
        }
        return color;
    }

    public void intakeMachine(boolean buttona, boolean buttonb, boolean buttonc, boolean buttond){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (intakeState){
            case UP:
                initPos();
                if(buttona){intakeState = DOWN;}
                if(buttonb){intakeState = LOOKING;}

                break;
            case DOWN:
                intakePos(0.5);
                intakeMotor.setPower(0);
                if(buttonb){intakeState = LOOKING;}
                if(buttonc){intakeState = UP;}
                if(buttond){intakeState = REJECT;}
                break;
            case LOOKING:
                intakePos(0.5);
                intakeMotor.setPower(-1);
                if(buttona){intakeState = DOWN;}
                if(buttonc){intakeState = UP;}
                if (getColor() == YELLOW || getColor() == RED || getColor() == BLUE){
                    intakeState = SECURED;
                }
                if(buttond){intakeState = REJECT;}
                break;
            case SECURED:
                intakePos(0.5);
                clawServo.setPosition(1);
                intakeState = TRANSFERED;
                break;
            case TRANSFERED:
                if (clawServo.getPosition() > 0.99) {
                    armUp();
                    Robot1Tele.extendoPosition = 0;
                }
                if(buttona){intakeState = DOWN;}
                if(buttonb){intakeState = LOOKING;}
                //if(buttond){intakeState = REJECT;}
                break;
            case REJECT:
                intakePos(0.5);
                intakeMotor.setPower(1);
                if(buttonb){intakeState = LOOKING;}
                if(buttonc){intakeState = UP;}
                if(buttona){intakeState = DOWN;}
        }
    }


    public enum IntakeState {
        DOWN, UP, LOOKING, SECURED, TRANSFERED, REJECT
    }


    public enum Color {
        BLUE, RED, YELLOW
    }




}
