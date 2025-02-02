package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.IntakeState.DOWN;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.IntakeState.LOOKING;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.IntakeState.REJECT;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.IntakeState.SECURED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.IntakeState.TRANSFERED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.SignColor.BLUESIGN;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.SignColor.NEUTRALSIGN;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake.SignColor.REDSIGN;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake;

public class v2CuttleIntake {
    public CRServo intakeMotor;
    public Servo leftServo, rightServo;
    public ColorRangeSensor signFinder;
    DigitalChannel pin0, pin1;

    CuttleServo lightbulb;
    public CuttleServo turntable;
    public CuttleServo clawServo;
    public v2CuttleIntake.IntakeState intakeState = UP;

    double clawInit = 1, clawGrab = 0.55, triggerTrigger = 0.1;

    double turntablePos = 0.5;
    public final double turntableInitPos = 0.5;

    public String colorLight;
    public v2CuttleIntake(CuttleServo claw, CuttleServo tt, HardwareMap hardwareMap, CuttleServo light, String color){
        leftServo = hardwareMap.get(Servo.class, "left intake");
        rightServo = hardwareMap.get(Servo.class, "right intake");
        clawServo = claw;
        turntable = tt;
        lightbulb = light;
        colorLight = color;
        intakeMotor = hardwareMap.get(CRServo.class,"intake");
        pin0 = hardwareMap.get(DigitalChannel.class, "digital0");
        pin1 = hardwareMap.get(DigitalChannel.class, "digital1");
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);
        signFinder = hardwareMap.get(ColorRangeSensor.class, "color");
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
    public void turntableCustom(double amount){
        intakePos(amount);
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
        color();
    }

    public void color(){
        if(colorLight == "red"){
            lightRed();
        }
        else if(colorLight == "blue"){
            lightBlue();
        }
        else{
            lightOff();
        }
    }
    public void armUp(){
        leftServo.setPosition(0.865);
        rightServo.setPosition(0.135);
        turntable.setPosition(turntableInitPos);
        intakeMotor.setPower(0);
    }

    public void armMiddle(){
        leftServo.setPosition(0.08);
        rightServo.setPosition(1-0.08);
    }
    //close claw pos 0.957
    public void intakePos(double turntableAngle){
        leftServo.setPosition(0.025);
        rightServo.setPosition(1-0.025);
        turntable.setPosition(turntableAngle);
        clawServo.setPosition(clawInit);
    }

    public void in(){
        intakeMotor.setPower(1);
    }
    public void off(){intakeMotor.setPower(0);}
    public void out(){intakeMotor.setPower(-1);}



    public Color getColor(){
        Color color = null;

        if (pin0.getState() && !pin1.getState()){
            color = BLUE;
        }
        if (!pin0.getState() && pin1.getState()){
            color = RED;
        }
        if (pin0.getState() && pin1.getState()){
            color = YELLOW;
        }
        return color;
    }

    public SignColor getSignColor(){
        SignColor signColor = null;

        if(signFinder.red() > signFinder.blue() && signFinder.getDistance(DistanceUnit.CM) < 2.5){
            signColor = REDSIGN;
        }
        else if(signFinder.blue() > signFinder.red() && signFinder.getDistance(DistanceUnit.CM) < 2.5){
            signColor = BLUESIGN;
        }
        else if(signFinder.getDistance(DistanceUnit.CM) > 2.5){
            signColor = NEUTRALSIGN;
        }
        /*
        else{
            color = null;
        }

         */
        return signColor;
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
                off();
                lightOff();
                turntablePos = turn * -0.2 + turntableInitPos;

                if(looking > triggerTrigger){intakeState = LOOKING;}
                if(up){intakeState = UP;}
                if(reject > triggerTrigger){intakeState = REJECT;}
                break;
            case LOOKING:
                intakePos(turntablePos);
                in();
                lightGreen();
                turntablePos = turn*-0.2 + turntableInitPos;

                if(down){intakeState = DOWN;}
                if(up){intakeState = UP;}
                if (getColor() == inColor || getColor() == YELLOW){
                    intakeState = SECURED;
                }
                if(reject > 0.1 || getColor() == rejectColor){intakeState = REJECT;}
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
                out();
                lightPurple();
                if(getColor() != YELLOW || getColor() != RED || getColor() != BLUE){intakeState = DOWN;}
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
                off();
                lightOff();
                turntablePos = turn * -0.2 + turntableInitPos;

                if(looking > triggerTrigger){intakeState = LOOKING;}
                if(up){intakeState = UP;}
                if(reject > triggerTrigger){intakeState = REJECT;}
                break;
            case LOOKING:
                intakePos(turntablePos);
                in();
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
                out();
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

    public enum SignColor {
        BLUESIGN, REDSIGN, NEUTRALSIGN
    }

    public void setIntakeState(IntakeState state){
        intakeState = state;
    }
}
