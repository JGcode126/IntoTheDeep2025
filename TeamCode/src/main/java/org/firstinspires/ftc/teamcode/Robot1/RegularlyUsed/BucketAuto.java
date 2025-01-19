package org.firstinspires.ftc.teamcode.Robot1.RegularlyUsed;

import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.Color.BLUE;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.Color.RED;
import static org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake.Color.YELLOW;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot1.Subsystems.CuttleSlides;

public class BucketAuto extends AutoSequence{
    private ElapsedTime timer;
    TaskManager manager;


    public BucketAuto(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, CuttleIntake intake, CuttleOutake outake,
                      Telemetry telemetry, TaskQueue queue, PTPController ptpController, MotorPositionController liftController,
                      MotorPositionController extController, CuttleExtendo extendo, CuttleSlides lift, CuttleDT dt, TaskManager manager) {
        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt);
        timer = new ElapsedTime();
        this.manager = manager;
    }
    public void park(){
        TaskList park = new TaskList();
        manager.task(park, () -> {
            intake.turntableMiddle();
            intake.armUp();
            extendoPosition = 0;
            liftPosition = 0;
            outake.parkPos();
        });

        manager.waypointTask(park, new Pose(-500, 1400, Math.toRadians(90)),0.9,0.5,100,false);

        manager.task(park, () -> {dt.drive(-0.5,0,0);});

        queue.addTask(park);
    }

    public void scoreSample(int finishxpos) {
        TaskList scoringSample = new TaskList();

        manager.task(scoringSample, () -> {
            liftPosition = 14;
            outake.scorePosMid();
        });

        manager.waypointTask(scoringSample, new Pose(-1000, 390, -0.7),0.8,0.6,10,false);

        manager.task(scoringSample, () -> {dt.drive(-0.2,0,0);});

        manager.delay(scoringSample, 700);

        manager.task(scoringSample, () -> {
            dt.drive(0,0,0);
            outake.openClaw();
        });

        manager.delay(scoringSample, 200);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });

        manager.waypointTask(scoringSample, new Pose(finishxpos, 330, 0),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }


    public void intakeSample(double x, double deg) {
        TaskList sample = new TaskList();

        manager.task(sample, () -> {
            timer.reset();
            intake.turntableMiddle();
            intake.in();
            extendoPosition = 0;
            liftPosition = 0;
        });

        manager.waypointTask(sample, new Pose(x, 330, Math.toRadians(deg)),0.9,0.1,10,false);

        manager.delay(sample, 400);

        sample.addTask(new CustomTask(() -> {
            boolean quit = false;
            extendoPosition = 5;
            intake.in();
            intake.clawOpen();
            intake.intakeDown();

            if (timer.seconds() > 3.5) {quit = true;}

            return intake.getColor() == YELLOW || intake.getColor() == RED || intake.getColor() == BLUE || quit;
        }));

        manager.task(sample, () -> {
            extendoPosition = 0;
            intake.clawClose();
        });

        queue.addTask(sample);
    }

    public void scoreFirstSample(int xPos, int yPos, int finishxpos) {
        TaskList scoringSample = new TaskList();

        manager.task(scoringSample, () -> {
            liftPosition = 14;
            outake.scorePosMid();
        });

        //x used to be -980, -1000 still works
        manager.waypointTask(scoringSample, new Pose(xPos, yPos, -0.7),0.6,0.6,10,false);

        manager.task(scoringSample, () -> {outake.openClaw();});

        manager.delay(scoringSample, 200);

        manager.task(scoringSample, () -> {
            liftPosition = 0;
            outake.readyPos();
        });

        manager.waypointTask(scoringSample, new Pose(finishxpos, 330, 0),0.6,0.1,10,false);

        queue.addTask(scoringSample);
    }

}
