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
    CuttleServo leftServo;
    CuttleServo rightServo;
    CuttleServo lightbulb;
    public CuttleServo turntable;
    public CuttleServo clawServo;
    public CuttleIntake.IntakeState intakeState = UP;

    double clawInit = 1, clawGrab = 0.55, triggerTrigger = 0.1;

    double turntablePos = 0.48;
    public final double turntableInitPos = 0.48;
    public CuttleIntake(CuttleServo left, CuttleServo right, CuttleServo claw, CuttleServo tt, HardwareMap hardwareMap, CuttleServo light){
        leftServo = left;
        rightServo = right;
        clawServo = claw;
        turntable = tt;
        lightbulb = light;
        intakeMotor = hardwareMap.get(CRServo.class,"intake");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");
    }

    public void lightRed(){
        lightbulb.setPosition(0.28);
    }
    public void lightOff(){
        lightbulb.setPosition(0);
    }
    public void lightBlue(){
        lightbulb.setPosition(0.61);
    }
    public void lightGreen(){
        lightbulb.setPosition(0.5);
    }
    public void lightPurple(){
        lightbulb.setPosition(0.7);
    }
    public void lightYellow(){
        lightbulb.setPosition(0.38);
    }


    public void intakeDown(){
        intakePos(0.5);
    }
    public void turntableRight(){
        intakePos(0.7);
    }
    public void turntableLeft(){
        intakePos(0.2);
    }
    public void turntableMiddle(){
        intakePos(turntableInitPos);
    }
    public void clawOpen(){
        clawServo.setPosition(clawInit);
    }
    public void clawClose(){
        clawServo.setPosition(clawGrab);
    }


    public void initPos(){
        armUp();
        clawServo.setPosition(clawInit);
    }

    public void armUp(){
        leftServo.setPosition(0.865);
        rightServo.setPosition(0.135);
        turntable.setPosition(turntableInitPos);
        intakeMotor.setPower(0);
    }
    //close claw pos 0.957
    public void intakePos(double turntableAngle){
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        turntable.setPosition(turntableAngle);
        clawServo.setPosition(clawInit);
    }

    public void in(){
        intakeMotor.setPower(-1);
    }
    public void off(){intakeMotor.setPower(0);}
    public void out(){intakeMotor.setPower(1);}

    public void nomnom (){
        if (getColor() == YELLOW || getColor() == RED || getColor() == BLUE){
            clawServo.setPosition(0.957);
            intakeMotor.setPower(0);
        }
    }

    public Color getColor(){
        Color color = null;
        colorSensor.enableLed(false);

        if(colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue() && colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = YELLOW;
        }
        else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green() && colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = BLUE;
        }
        else if(colorSensor.blue() < colorSensor.red() && colorSensor.red() > colorSensor.green() && colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = RED;
        }
        else if(colorSensor.getDistance(DistanceUnit.CM) < 2.5){
            color = BLUE;
        }
        /*
        else{
            color = null;
        }

         */
        return color;
    }

    public void intakeMachineColor(boolean down, double looking, boolean up, double reject, double turn, Color inColor, Color rejectColor){

        switch (intakeState){
            case UP:
                initPos();
                turntablePos = turntableInitPos;
                lightOff();
                if(down){intakeState = DOWN;}
                if(looking > triggerTrigger){intakeState = LOOKING;}
                break;
            case DOWN:
                intakePos(turntablePos);
                intakeMotor.setPower(0);
                lightOff();
                turntablePos = turn * -0.2 + turntableInitPos;

                if(looking > triggerTrigger){intakeState = LOOKING;}
                if(up){intakeState = UP;}
                if(reject > triggerTrigger){intakeState = REJECT;}
                break;
            case LOOKING:
                intakePos(turntablePos);
                intakeMotor.setPower(-1);
                lightGreen();
                turntablePos = turn*-0.2 + turntableInitPos;

                if(down){intakeState = DOWN;}
                if(up){intakeState = UP;}
                if (getColor() == YELLOW || getColor() == inColor){intakeState = SECURED;}
                if (getColor() == rejectColor){intakeState = REJECT;}
                if(reject > 0.1){intakeState = REJECT;}
                break;
            case SECURED:
                intakePos(turntableInitPos);
                clawServo.setPosition(clawGrab);
                lightOff();
                intakeState = TRANSFERED;
                break;
            case TRANSFERED:
                if (clawServo.getPosition() > clawGrab - 0.015) {
                    armUp();
                    off();
                }
                if(down){intakeState = DOWN;}
                if(looking > triggerTrigger){intakeState = LOOKING;}
                if(up){intakeState = UP;}
                break;
            case REJECT:
                intakePos(turntableInitPos);
                intakeMotor.setPower(1);
                lightPurple();
                if(getColor() != YELLOW || getColor() != inColor){intakeState = DOWN;}
                if(getColor() == rejectColor){intakeState = REJECT;}
                if(up){intakeState = UP;}
                if(down){intakeState = DOWN;}
        }
    }

    public void intakeMachine(boolean down, double looking, boolean up, double reject, double turn){
        switch (intakeState){
            case UP:
                initPos();
                turntablePos = turntableInitPos;
                lightOff();
                if(down){intakeState = DOWN;}
                if(looking > triggerTrigger){intakeState = LOOKING;}
                break;
            case DOWN:
                intakePos(turntablePos);
                intakeMotor.setPower(0);
                lightOff();
                turntablePos = turn * -0.2 + turntableInitPos;

                if(looking > triggerTrigger){intakeState = LOOKING;}
                if(up){intakeState = UP;}
                if(reject > triggerTrigger){intakeState = REJECT;}
                break;
            case LOOKING:
                intakePos(turntablePos);
                intakeMotor.setPower(-1);
                lightGreen();
                turntablePos = turn*-0.2 + turntableInitPos;

                if(down){intakeState = DOWN;}
                if(up){intakeState = UP;}
                if (getColor() == YELLOW || getColor() == RED || getColor() == BLUE){
                    intakeState = SECURED;
                }
                if(reject > 0.1){intakeState = REJECT;}
                break;
            case SECURED:
                intakePos(turntableInitPos);
                clawServo.setPosition(clawGrab);
                lightOff();
                intakeState = TRANSFERED;
                break;
            case TRANSFERED:
                if (clawServo.getPosition() > clawGrab - 0.015) {
                    armUp();
                    off();
                }
                if(down){intakeState = DOWN;}
                if(looking > triggerTrigger){intakeState = LOOKING;}
                if(up){intakeState = UP;}
                break;
            case REJECT:
                intakePos(turntableInitPos);
                intakeMotor.setPower(1);
                lightPurple();
                if(getColor() != YELLOW || getColor() != RED || getColor() != BLUE){intakeState = DOWN;}
                if(up){intakeState = UP;}
                if(down){intakeState = DOWN;}
        }
    }

    

    public enum IntakeState {
        DOWN, UP, LOOKING, SECURED, TRANSFERED, REJECT
    }


    public enum Color {
        BLUE, RED, YELLOW
    }

    public void setIntakeState(IntakeState state){
        intakeState = state;
    }




}
