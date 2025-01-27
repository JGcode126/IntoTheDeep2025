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
                        v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager, TeleOp teleOp/*, SpecimenAuto s, BucketAuto b*/) {

        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt);
        this.manager = manager;
        this.teleOp = teleOp;

        this.specimen = new SpecimenAuto(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt, new TaskManager(queue, ptpController));
        this.bucket = new BucketAuto(otosLocalizer, encoderLocalizer, intake, outake, telemetry, queue,
                ptpController, liftPosController, extendoPosController, extendo, lift, dt,
                new TaskManager(queue, ptpController));

    }
    public AutoSequence(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer,
                        v2CuttleIntake intake, v2CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                        PTPController ptpController, MotorPositionController liftController,
                        MotorPositionController extController, v2CuttleExtendo extendo,
                        v2CuttleSlides lift, v2CuttleDT dt) {

        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt);
    }

    public void testTele(){/*teleOp.testingIfWorks();*/}

    public void scoringBuckets(int xpos, int xfinishpos, int deg){
        bucket.intakeSample(xpos, deg);
        //teleOp.transferSequence();
        bucket.scoreSample(xpos);
    }
    public void scoringSpec3Fancy(){
        specimen.scoringSpecimenFancy(0, -4, 0,0,0);
        specimen.scoringSpecimenFancy(0.25, -6,0,0,20);
        specimen.scoringSpecimenFancy(0.75,-7,0, 0,50);
    }

    public void allSamples(){
        specimen.ttSample(-580, -720,90,-850,-600, 50, -0.25);//first sample
        specimen.ttSample(-870, -720,90,-1000,-550, 50, -0.25);//second sample
        specimen.ttSample(2,4, -1180, -740,110,-850,-400, 50, -0.25);//third sample

    }

    public void startSpecimen(int distance, double height, double speed1, double speed2){
        TaskList scoreSpec = new TaskList();

        manager.task(scoreSpec, () ->{
            liftPosition = height;
            outake.autoAutoHighRungPos();
            extendoPosition = 3;
        });

        manager.waypointTask(scoreSpec, new Pose(-100, distance, 0),speed1,0.5,150,false);

        manager.delay(scoreSpec, 200);

        manager.task(scoreSpec, () ->{
            outake.openClaw();
            liftPosition = height-1;
            extendoPosition = 3;
        });

        manager.delay(scoreSpec, 300);

        //first sample values: x = -600, y = -720, r = 90
        //old values x = -150, y = -400, r = 50
        manager.waypointTask(scoreSpec,  new Pose(-500, -850, Math.toRadians(70)),speed2,0.5,150,false);

        manager.task(scoreSpec, () ->{
            outake.transferPos();
            liftPosition = 0;
            extendoPosition = 0;
            outake.readyPos();
            intake.intakeDown();
        });

        queue.addTask(scoreSpec);
    }
}
