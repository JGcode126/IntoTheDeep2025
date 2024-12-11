package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
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

public class Setup extends CuttleInitOpMode {
    public Setup(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, CuttleIntake intake, CuttleOutake outake, Telemetry telemetry, TaskQueue queue,
                                     PTPController ptpController, MotorPositionController liftController, MotorPositionController extController,
                                     CuttleExtendo extendo, CuttleSlides lift, CuttleDT dt) {
        //Initializing values
        this.otosLocalizer = otos;
        this.encoderLocalizer = encoderLocalizer;
        this.intake = intake;
        this.outake = outake;
        this.telemetry = telemetry;
        this.queue = queue;
        this.ptpController = ptpController;
        this.liftPosController = liftController;
        this.extendoPosController = extController;
        this.lift = lift;
        this.extendo = extendo;
        this.dt = dt;
    }

    //To initialize the robot for start
    public void initRobot() {
        otosLocalizer.reset();//reset otos
        encoderLocalizer.reset();//reset odo

        //inti intake and outake
        outake.initAutoPos();
        intake.initPos();

        //set slide home
        liftPosController.setHome();
        extendoPosController.setHome();
    }

    public void telemetryData() {
        //Printing out data
        //works!!
        otosLocalizer.update();//updating otos values
        Pose currentPos = otosLocalizer.getPos();//setting position of otos to current pos

        telemetry.addData("Intake Color", intake.getColor());//print out intake color

        //print out oto values
        telemetry.addData("Cuttle X:", currentPos.getX());
        telemetry.addData("Cuttle Y:", currentPos.getY());
        telemetry.addData("Cuttle R:", currentPos.getR());

        telemetry.update();
    }

    //I don't think this is needed
    public void addSequence(Runnable taskMethod) {
        queue.addTask(new CustomTask(() -> {
            taskMethod.run();
            return true;
        }));
    }

    //Add position task
    public void addWaypointTask(TaskList taskList, Pose pose) {
        //takes in the task and position
        //adds the position to the given taskList
        taskList.addTask(new PointTask(new Waypoint(pose) , ptpController));
    }

    //Constructer gets in other stuff too -- power, rSlop, tSlop, passthrough
    //More detailed driving
    //Made this to not mess up my code
    public void addWaypointTask(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        //takes in the task and position
        //adds the position to the given taskList
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), ptpController));
    }

    //To add a delay task
    public void addDelayTask(TaskList taskList, int delay) {
        //takes delay and taskList
        //adds delay to task list
        taskList.addTask(new DelayTask(delay));
        //idk if this is needed
    }

    //Intake task
    //Also don't think this is needed
    public void addIntakeTask(TaskList taskList, Runnable intakeAction) {
        taskList.addTask(new CustomTask(() -> {
            intakeAction.run();
            return true;
        }));
    }
}
