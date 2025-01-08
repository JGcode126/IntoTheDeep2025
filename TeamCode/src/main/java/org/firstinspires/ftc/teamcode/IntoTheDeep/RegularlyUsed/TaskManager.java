package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.ForkTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.Task;
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

//Class that just maneges the tasks
public class TaskManager{
    TaskQueue queue;
    PTPController ptpController;

    public TaskManager(TaskQueue queue, PTPController ptpController) {
        this.queue = queue;
        this.ptpController = ptpController;
    }

    public void forkTask(Task task1, Task task2) {//fork tasks, will this work??
        //class ForkTask(val task1: Task, val task2: Task) : Task
        queue.addTask(new ForkTask(task1, task2));
    }

    //to add a task
    public void addTask(Task task) {//got lazy and didn't want to change the TeleOp class code
        queue.addTask(task);
    }

    //Add position task (basic)
    public void waypointTask(TaskList task, Pose pose) {task.addTask(new PointTask(new Waypoint(pose) , ptpController));}

    //if want to switch through oto and odo
    public void waypointTask(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough, PTPController controller) {
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), controller));
    }
    //Gets in other stuff too -- power, rSlop, tSlop, passthrough
    public void waypointTask(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), ptpController));
    }

    public void delay(TaskList taskList, int delay) {taskList.addTask(new DelayTask(delay));}//To add a delay task

    public void task(TaskList taskList, Runnable action){//to add a bunch of tasks
        taskList.addTask(new CustomTask(() -> {
            action.run();
            return true;
        }));
    }
}
