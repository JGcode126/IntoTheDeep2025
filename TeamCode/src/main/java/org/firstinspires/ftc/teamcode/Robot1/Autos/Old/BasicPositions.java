package org.firstinspires.ftc.teamcode.Robot1.Autos.Old;

import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleExtendo.ExtendoState.INE;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.IntakeState.UP;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake.OutakeState.BUCKET_BAR;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleSlides.LiftState.IN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Robot1.Init.CuttleInitOpMode;

@Autonomous
@Disabled
public class BasicPositions extends CuttleInitOpMode {
    double finalExtendoPos = 0;
    double finalLiftPos = 0;
    double counter = 0;
    public boolean transfering = false;

    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        encoderLocalizer.reset();

        liftPosition = 0;
        extendoPosition = 0;

        intake.initPos();
        liftPosController.setHome();

        outake.closeClaw();
        //outake.autoPos();
    }

    public void main(){
        super.main();

        queue.addTask(new CustomTask(()->{
            liftPosition = 3;
            outake.autoHighRungPos();
            return true;
        }));


        queue.addTask(new PointTask(new Waypoint(new Pose(0,-850,0)), ptpController));

        queue.addTask(new DelayTask(500));
        queue.addTask(new CustomTask(()->{
            outake.openClaw();
            liftPosition = 2.5;
            return true;
        }));

        queue.addTask(new PointTask(new Waypoint(new Pose(0,-400, 0)), ptpController));

        queue.addTask(new CustomTask(()->{
            outake.transferPos();
            liftPosition = 0;
            return true;
        }));
        queue.addTask(new PointTask(new Waypoint(new Pose(1400,-400, Math.toRadians(-205))), ptpController));

        queue.addTask(new CustomTask(()->{
            intake.intakePos(0.5);
            intake.in();
            return true;
        }));
        queue.addTask(new DelayTask(500));

        queue.addTask(new CustomTask(()->{
            extendoPosition = 5;
            return true;
        }));

        queue.addTask(new DelayTask(5000));

        queue.addTask(new CustomTask(()->{
            intake.off();
            intake.armUp();
            extendoPosition = 0;
            return true;
        }));

        queue.addTask(new CustomTask(()->{
            transferSequence();
            return true;
        }));


        queue.addTask(new PointTask(new Waypoint(new Pose(0,-0, 0)), ptpController));

        /*
        queue.addTask(new CustomTask(()->{
            intake.off();
            intake.armUp();
            extendoPosition = 0;
            return true;
        }));

        queue.addTask(new PointTask(new Waypoint(new Pose(1500,-500, Math.toRadians(-190))), ptpController));
        queue.addTask(new DelayTask(1000));


        queue.addTask(new CustomTask(()->{
            intake.intakePos(0.5);
            intake.in();
            return true;
        }));
        queue.addTask(new DelayTask(500));

        queue.addTask(new CustomTask(()->{
            extendoPosition = 4;
            return true;
        }));

        queue.addTask(new DelayTask(2000));
        queue.addTask(new CustomTask(()->{
            intake.off();
            intake.armUp();
            extendoPosition = 0;
            return true;
        }));

        queue.addTask(new PointTask(new Waypoint(new Pose(1500,-400, Math.toRadians(-175))), ptpController));
        queue.addTask(new DelayTask(1000));


        queue.addTask(new CustomTask(()->{
            //turn angle of intake too
            //rotate angle of intake to side
            intake.intakePos(0.5);
            intake.in();
            return true;
        }));
        queue.addTask(new DelayTask(500));

        queue.addTask(new CustomTask(()->{
            extendoPosition = 4;
            return true;
        }));

        queue.addTask(new DelayTask(2000));
        queue.addTask(new CustomTask(()->{
            intake.off();
            intake.armUp();
            extendoPosition = 0;
            return true;
        }));

        queue.addTask(new DelayTask(5000));
        queue.addTask(new PointTask(new Waypoint(new Pose(0,0,0)), ptpController));*/
    }

    public void mainLoop(){
        super.mainLoop();

        encoderLocalizer.update();
        System.out.println(encoderLocalizer.getPos());
        telemetry.addData("Cuttle X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Cuttle Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Cuttle R:",encoderLocalizer.getPos().getR());
        telemetry.addData("otos r", pos.h);
        telemetry.addData("otos x", pos.x);
        telemetry.addData("otos y", pos.y);
        telemetry.update();
    }

    //A work in progress
    //this runs at the end after all the code runs

    void transferSequence(){
        TaskList transfer = new TaskList();
        intake.setIntakeState(UP);
        lift.setLiftState(IN);
        transfer.addTask(new CustomTask(()->{
            counter += 1;
            intake.armUp();
            intake.clawServo.setPosition(0.45);
            outake.readyPos();
            finalExtendoPos = 0;
            finalLiftPos = 0;
            telemetry.addData("tranfer sequence running", true);
            return true;
        }));
        transfer.addTask(new DelayTask(300));
        transfer.addTask(new CustomTask(()->{
            //finalSlidePos = extendo.extendoMachine(true, false, false);
            outake.transferPos();
            return true;
        }));
        transfer.addTask(new DelayTask(100));
        transfer.addTask(new CustomTask(()->{
            outake.grippedPos();
            intake.initPos();
            intake.setIntakeState(UP);
            return true;
        }));
        transfer.addTask(new DelayTask(200));
        transfer.addTask(new CustomTask(()->{
            outake.scorePosMid();
            finalExtendoPos = 1;
            return true;
        }));
        transfer.addTask(new DelayTask(200));
        transfer.addTask(new CustomTask(()->{
            finalExtendoPos = 0;
            outake.setScoreState(BUCKET_BAR);
            extendo.setExtendoState(INE);

            transfering = false;
            return true;
        }));

        queue.addTask(transfer);
    }

    public void pickingUpYellows(){
        queue.addTask(new CustomTask(()->{
            queue.addTask(new CustomTask(()->{
                intake.intakePos(0.5);
                intake.in();
                return true;
            }));
            queue.addTask(new DelayTask(500));

            queue.addTask(new CustomTask(()->{
                extendoPosition = 5;
                return true;
            }));

            queue.addTask(new DelayTask(2000));
            queue.addTask(new CustomTask(()->{
                intake.off();
                intake.armUp();
                extendoPosition = 0;
                return true;
            }));
            return true;
        }));
    }
}
