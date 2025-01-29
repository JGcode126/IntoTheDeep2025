package org.firstinspires.ftc.teamcode.Robot_V2.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.BACKINTAKE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.BARLEFT;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.BARRIGHT;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.FRONTSCORE;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.GRIPPED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.HOLD;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.PLACED;
import static org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake.OutakeState.READY;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake;

public class v2CuttleOutake {
    CuttleServo claw;
    public Servo driveRight, driveLeft, wrist;
    public v2CuttleOutake.OutakeState outakeState = READY;
    double readyCounter = 0;

    public v2CuttleOutake(CuttleServo clawServo, HardwareMap hardwareMap){
        driveRight = hardwareMap.get(Servo.class, "right outtake");;
        driveLeft = hardwareMap.get(Servo.class, "left outtake");
        claw = clawServo;
        wrist = hardwareMap.get(Servo.class, "outtake wrist");;
    }

    //TODO: Delete this??
    public void autoAutoHighRungPos(){
        driveRight.setPosition(0.4);
        driveLeft.setPosition(1-0.4);
        wristCenter();
    }

    //For testing
    public void setPos(double pos){
        driveRight.setPosition(pos);
        driveLeft.setPosition(1-pos);
        wristCenter();
    }

    public void autoHighRungPos(){
        driveRight.setPosition(0.38);
        driveLeft.setPosition(1-0.38);
        wristCenter();
    }

    public void parkPos(){
        driveRight.setPosition(0.35);
        driveLeft.setPosition(1-0.35);
        wristCenter();
    }

    public void initAutoPos(){
        driveRight.setPosition(0.80);
        driveLeft.setPosition(1-0.8);
        closeClaw();
    }

    public void readyPos(){
        wristCenter();
        driveRight.setPosition(0.85);
        driveLeft.setPosition(1-0.85);
        openClaw();
    }

    public void midHolding(){
        closeClaw();
        driveRight.setPosition(0.3);
        driveLeft.setPosition(1-0.3);
        wristCenter();
    }

    public void transferPos(){
        wristCenter();
        driveRight.setPosition(0.95);
        driveLeft.setPosition(1-0.95);
        openClaw();
    }

    public void grippedPos(){
        closeClaw();
        driveRight.setPosition(0.95);
        driveLeft.setPosition(1-0.95);
        wristCenter();
    }
    public void backIntakePos(){
        openClaw();
        driveRight.setPosition(0.15);
        driveLeft.setPosition(1-0.15);
        wristCenter();
    }

    public void specimenFrontReadyPos(){
        closeClaw();
        driveRight.setPosition(0.80);
        driveLeft.setPosition(1-0.8);
        wristCenter();
    }



    public void closeClaw(){
        claw.setPosition(0.5);
    }

    public void openClaw(){
        claw.setPosition(0.1);
    }

    public void wristRight(){
        wrist.setPosition(0.95);
    }

    public void wristLeft(){
        wrist.setPosition(0.38);
    }

    public void wristCenter(){
        wrist.setPosition(0.12);
    }
    public void wristDown(){
        wrist.setPosition(0.7);
    }

    public void scorePosMid(){
        driveRight.setPosition(0.5);
        driveLeft.setPosition(1-0.5);
        closeClaw();
        wristDown();
    }

    public void scorePosRight(){
        driveRight.setPosition(0.4);
        driveLeft.setPosition(1-0.4);
        closeClaw();
        wristRight();
    }

    public void scorePosLeft(){
        driveRight.setPosition(0.4);
        driveLeft.setPosition(1-0.4);
        closeClaw();
        wristLeft();
    }




    public void outakeMachine(boolean ready, boolean place, boolean grip, boolean holding, boolean bucket_bar, boolean barLeft, boolean barRight, boolean back, boolean backTransfer){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (outakeState){
            case READY:
                readyPos();
                openClaw();
                if(place){outakeState = PLACED;}
                if (back) {outakeState = BACKINTAKE;}
                break;
            case PLACED:
                transferPos();
                if(grip){outakeState = GRIPPED;}
                if(ready){outakeState = READY;}
                break;
            case GRIPPED:
                grippedPos();
                if(ready){outakeState = READY;}
                if(bucket_bar){
                    readyCounter = 0;
                    outakeState = BUCKET_BAR;
                }
                if(barLeft){outakeState = BARLEFT;}
                if(barRight){outakeState = BARRIGHT;}
                if(holding){outakeState = HOLD;}
                break;
            case HOLD:
                midHolding();
                if(bucket_bar){
                    readyCounter = 0;
                    outakeState = BUCKET_BAR;
                }
                if(barLeft){outakeState = BARLEFT;}
                if(barRight){outakeState = BARRIGHT;}
                break;
            case BUCKET_BAR:
                scorePosMid();
                if(ready){
                    readyCounter += 1;
                    openClaw();
                }
                if (readyCounter > 5) {
                    outakeState = READY;
                    readyCounter = 0;
                }
                if(grip){outakeState = GRIPPED;}
                if(barLeft){outakeState = BARLEFT;}
                if(barRight){outakeState = BARRIGHT;}
                break;
            case BARRIGHT:
                scorePosRight();
                if(ready){
                    openClaw();
                    outakeState = READY;
                }
                if(barLeft){outakeState = BARLEFT;}
                if(bucket_bar){
                    readyCounter = 0;
                    outakeState = BUCKET_BAR;
                }
                if(grip){outakeState = GRIPPED;}
                break;
            case BARLEFT:
                scorePosLeft();
                if(ready){
                    openClaw();
                    outakeState = READY;
                }
                if(barRight){outakeState = BARRIGHT;}
                if(bucket_bar){
                    readyCounter = 0;
                    outakeState = BUCKET_BAR;
                }
                if(grip){outakeState = GRIPPED;}
                break;
            case BACKINTAKE:
                backIntakePos();
                if(backTransfer){
                    readyCounter += 1;
                    closeClaw();
                }
                if (readyCounter > 5) {
                    outakeState = FRONTSCORE;
                    readyCounter = 0;
                }
                break;
            case FRONTSCORE:
                specimenFrontReadyPos();
                break;
        }
    }




    public enum OutakeState {
        READY, PLACED, GRIPPED, BUCKET_BAR, BARRIGHT, BARLEFT, HOLD, BACKINTAKE, FRONTSCORE
    }

    public void setScoreState(OutakeState state){
        outakeState = state;
    }





}
