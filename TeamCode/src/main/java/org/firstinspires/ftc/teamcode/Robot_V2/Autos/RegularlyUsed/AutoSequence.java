package org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed;

import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleSlides;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleDT;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleHang;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides;

public class AutoSequence extends Setup {
    SpecimenAuto specimen;
    BucketAuto bucket;
    TaskManager manager;

    public AutoSequence(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer,
                        v2CuttleIntake intake, v2CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                        PTPController ptpController, MotorPositionController liftController,
                        MotorPositionController extController, v2CuttleExtendo extendo,
                        v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager, TeleOp teleOp, v2CuttleHang hang) {

        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt, hang);
        this.manager = manager;
        this.teleOp = teleOp;

        this.specimen = new SpecimenAuto(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt, new TaskManager(queue, ptpController), hang);
        this.bucket = new BucketAuto(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt,
                new TaskManager(queue, ptpController), hang);

    }

    public AutoSequence(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer,
                        v2CuttleIntake intake, v2CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                        PTPController ptpController, MotorPositionController liftController,
                        MotorPositionController extController, v2CuttleExtendo extendo,
                        v2CuttleSlides lift, v2CuttleDT dt, v2CuttleHang hang) {

        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt, hang);
    }

    public void testDriveFusion(int x, int y, double r, double speed){
        TaskList drive = new TaskList();

        manager.waypointTaskFusion(drive, new Pose(x,y, Math.toRadians(r)), speed, 0.5, 150, false);
    }

    public void testDriveOtos(int x, int y, double r, double speed){
        TaskList drive = new TaskList();

        manager.waypointTaskOtos(drive, new Pose(x,y, Math.toRadians(r)), speed, 0.5, 150, false);
    }

    public void testScoring(double highChamberPos) {
        TaskList transfer = new TaskList();

        manager.task(transfer, () -> {
            extendoPosition = 1;
            outake.scorePosLeft();
            liftPosition = highChamberPos;
        });

        manager.addTask(transfer);
    }

    public void startSpecimen(int x, int distance, double height, double speed1, double speed2, int x2,int y2,int r2, double extPos){
        TaskList scoreSpec = new TaskList();

        manager.task(scoreSpec, () ->{
            liftPosition = height;
            outake.autoHighRungPos();
            extendoPosition = 3;
        });

        manager.waypointTask(scoreSpec, new Pose(x, distance, 0),speed1,0.5,150,false);

        manager.delay(scoreSpec, 200);

        manager.task(scoreSpec, () ->{
            outake.openClaw();
            liftPosition = height-1;
            extendoPosition = extPos;
            intake.intakeDown();
            intake.turntableRight();
        });

        manager.delay(scoreSpec, 300);

        //first sample values: x = -600, y = -720, r = 90
        //old values x = -150, y = -400, r = 50
        manager.waypointTask(scoreSpec,  new Pose(x2, y2, Math.toRadians(r2)),speed2,0.5,150,false);

        manager.task(scoreSpec, () ->{
            outake.transferPos();
            liftPosition = 0;
            extendoPosition = extPos;
            outake.readyPos();
            intake.intakeDown();
            intake.in();
        });

        queue.addTask(scoreSpec);
    }


}
