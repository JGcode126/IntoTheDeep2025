package org.firstinspires.ftc.teamcode.IntoTheDeep.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;

@Autonomous
public class BasicPositions extends CuttleInitOpMode {
    public void onInit(){
        super.onInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        encoderLocalizer.reset();

        liftPosition = 0;//Doesn't work
        extendoPosition = 0;//this too

        intake.initPos();
    }

    public void main(){
        super.main();
        /*queue.addTask(new CustomTask(()->{
            liftPosition = 5;
            return true;
        }));*/
        //queue.addTask(new PointTask(new Waypoint(new Pose(0,0,Math.toRadians(-180))), ptpController));
        queue.addTask(new PointTask(new Waypoint(new Pose(0,-600,0)), ptpController));
        queue.addTask(new DelayTask(1000));
        queue.addTask(new PointTask(new Waypoint(new Pose(1400,-400, Math.toRadians(-205))), ptpController));
        queue.addTask(new DelayTask(1000));

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
        queue.addTask(new PointTask(new Waypoint(new Pose(0,0,0)), ptpController));
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
