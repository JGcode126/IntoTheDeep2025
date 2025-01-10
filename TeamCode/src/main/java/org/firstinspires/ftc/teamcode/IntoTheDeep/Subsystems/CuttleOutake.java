package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BACKINTAKE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BARLEFT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BARRIGHT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.FRONTSCORE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.GRIPPED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.HOLD;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.PLACED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.READY;

import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

public class CuttleOutake {
    CuttleServo driveRight, driveLeft, claw, wrist;
    public CuttleOutake.OutakeState outakeState = READY;

    public CuttleOutake(CuttleServo driveServoRight, CuttleServo wristServo, CuttleServo clawServo, CuttleServo driveServoLeft){
        driveRight = driveServoRight;
        driveLeft = driveServoLeft;
        claw = clawServo;
        wrist = wristServo;
    }

    public void autoHighRungPos(){
        driveRight.setPosition(0.19);
        driveLeft.setPosition(1-0.19);
    }

    public void autoAutoHighRungPos(){
        driveRight.setPosition(0.18);
        driveLeft.setPosition(1-0.18);
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
        driveRight.setPosition(0.60);
        driveLeft.setPosition(1-0.6);
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
        driveRight.setPosition(1);
        driveLeft.setPosition(0);
        wristCenter();
    }
    public void backIntakePos(){
        openClaw();
        driveRight.setPosition(0);
        driveLeft.setPosition(1);
        wristCenter();
    }

    public void specimenFrontReadyPos(){
        closeClaw();
        driveRight.setPosition(0.80);
        driveLeft.setPosition(1-0.8);
        wristDown();
    }



    public void closeClaw(){
        claw.setPosition(0.49);
    }

    public void openClaw(){
        claw.setPosition(0.7);
    }

    public void wristRight(){
        wrist.setPosition(0.82);
    }

    public void wristLeft(){
        wrist.setPosition(0.25);
    }

    public void wristCenter(){
        wrist.setPosition(0.52);
    }
    public void wristDown(){
        wrist.setPosition(0);
    }

    public void scorePosMid(){
        driveRight.setPosition(0.3);
        driveLeft.setPosition(1-0.3);
        closeClaw();
        wristDown();
    }

    public void scorePosRight(){
        driveRight.setPosition(0.19);
        driveLeft.setPosition(1-0.19);
        closeClaw();
        wristRight();
    }

    public void scorePosLeft(){
        driveRight.setPosition(0.19);
        driveLeft.setPosition(1-0.19);
        closeClaw();
        wristLeft();
    }




    public void outakeMachine(boolean ready, boolean place, boolean grip, boolean holding, boolean bucket_bar, boolean barLeft, boolean barRight, boolean back, boolean backTransfer){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (outakeState){
            case READY:
                if (claw.getPosition() > 0.45) {
                    readyPos();
                } else {
                    openClaw();
                }
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
                if(bucket_bar){outakeState = BUCKET_BAR;}
                if(barLeft){outakeState = BARLEFT;}
                if(barRight){outakeState = BARRIGHT;}
                if(holding){outakeState = HOLD;}
                break;
            case HOLD:
                midHolding();
                if(bucket_bar){outakeState = BUCKET_BAR;}
                if(barLeft){outakeState = BARLEFT;}
                if(barRight){outakeState = BARRIGHT;}
                break;
            case BUCKET_BAR:
                scorePosMid();
                if(ready){
                    openClaw();
                    outakeState = READY;
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
                if(bucket_bar){outakeState = BUCKET_BAR;}
                if(grip){outakeState = GRIPPED;}
                break;
            case BARLEFT:
                scorePosLeft();
                if(ready){
                    openClaw();
                    outakeState = READY;
                }
                if(barRight){outakeState = BARRIGHT;}
                if(bucket_bar){outakeState = BUCKET_BAR;}
                if(grip){outakeState = GRIPPED;}
                break;
            case BACKINTAKE:
                backIntakePos();
                if (backTransfer){outakeState = FRONTSCORE;}
                break;
            case FRONTSCORE:
                specimenFrontReadyPos();
                if(ready){
                    openClaw();
                    outakeState = READY;
                }
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
