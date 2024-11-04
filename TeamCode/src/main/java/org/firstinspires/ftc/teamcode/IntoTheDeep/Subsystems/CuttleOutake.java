package org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.DOWN;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.LOOKING;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.REJECT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.SECURED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.TRANSFERED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BARLEFT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BARRIGHT;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.GRIPPED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.HOLD;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.PLACED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake.OutakeState.READY;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.OpModes.Robot1Tele;

public class CuttleOutake {
    CuttleServo drive, claw, wrist;
    public CuttleOutake.OutakeState outakeState = READY;

    public CuttleOutake(CuttleServo driveServo, CuttleServo wristServo, CuttleServo clawServo){
        drive = driveServo;
        claw = clawServo;
        wrist = wristServo;
    }

    public void readyPos(){
        wristCenter();
        drive.setPosition(0.65);
        openClaw();
    }

    public void midHolding(){
        closeClaw();
        drive.setPosition(0.3);
        wristCenter();
    }

    public void transferPos(){
        wristCenter();
        drive.setPosition(0.8);
        openClaw();
    }

    public void grippedPos(){
        closeClaw();
        drive.setPosition(0.8);
        wristCenter();
    }

    public void closeClaw(){
        claw.setPosition(0.4);
    }

    public void openClaw(){
        claw.setPosition(0.7);
    }

    public void wristRight(){
        wrist.setPosition(0.75);
    }

    public void wristLeft(){
        wrist.setPosition(0.25);
    }

    public void wristCenter(){
        wrist.setPosition(0.5);
    }
    public void wristDown(){
        wrist.setPosition(0);
    }

    public void scorePosMid(){
        drive.setPosition(0.25);
        closeClaw();
        wristDown();
    }

    public void scorePosRight(){
        drive.setPosition(0.25);
        closeClaw();
        wristRight();
    }

    public void scorePosLeft(){
        drive.setPosition(0.25);
        closeClaw();
        wristLeft();
    }




    public void outakeMachine(boolean ready, boolean place, boolean grip, boolean holding, boolean bucket_bar, boolean barLeft, boolean barRight){
        //buttona: right trigger 2, buttonb: left trigger 2, buttonc: x 1, buttond: o 1, buttone: triangle 1
        switch (outakeState){
            case READY:
                if (claw.getPosition() > 0.45) {
                    readyPos();
                } else {
                    openClaw();
                }
                if(place){outakeState = PLACED;}
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
        }
    }




    public enum OutakeState {
        READY, PLACED, GRIPPED, BUCKET_BAR, BARRIGHT, BARLEFT, HOLD
    }

    public void setScoreState(OutakeState state){
        outakeState = state;
    }





}
