package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake.Color.YELLOW;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;

public class AutoSequence extends Setup{
    SpecimenAuto specimen;
    BucketAuto bucket;
    TaskManager manager;

    public AutoSequence(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer,
                        CuttleIntake intake, CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                        PTPController ptpController, MotorPositionController liftController,
                        MotorPositionController extController, CuttleExtendo extendo,
                        CuttleSlides lift, CuttleDT dt, TaskManager manager, TeleOp teleOp/*, SpecimenAuto s, BucketAuto b*/) {

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
                        CuttleIntake intake, CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                        PTPController ptpController, MotorPositionController liftController,
                        MotorPositionController extController, CuttleExtendo extendo,
                        CuttleSlides lift, CuttleDT dt) {

        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt);
    }

    public void testTele(){
        teleOp.testingIfWorks();
    }

    public void scoringBuckets(int xpos, int xfinishpos, int deg){
        bucket.intakeSample(xpos, deg);
        teleOp.transferSequence();
        bucket.scoreSample(xpos);
    }

    public void scoringSpec3(){
        specimen.scoringSpecimen(0, -4, 0,0,0);
        specimen.scoringSpecimen(0.25, -6,0,0,20);
        specimen.scoringSpecimen(0.75,-7,0, 0,50);
    }
    public void scoringSpec3Fancy(){
        specimen.scoringSpecimenFancy(0, -4, 0,0,0);
        specimen.scoringSpecimenFancy(0.25, -6,0,0,20);
        specimen.scoringSpecimenFancy(0.75,-7,0, 0,50);
    }

    public void allSamples(){
        specimen.ttSample(-600, -720,90,-850,-400, 70, -0.12);//first sample
        specimen.ttSample(-920, -720,90,-850,-400, 70, -0.12);//second sample
        specimen.ttSample(-1230, -610,100,-850,-400, 70, -0.12);//third sample
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
            liftPosition = 2;
            extendoPosition = 3;
        });

        manager.delay(scoreSpec, 300);

        //first sample values: x = -600, y = -720, r = 90
        //old values x = -150, y = -400, r = 50
        manager.waypointTask(scoreSpec,  new Pose(-500, -720, Math.toRadians(70)),speed2,0.5,150,false);

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
