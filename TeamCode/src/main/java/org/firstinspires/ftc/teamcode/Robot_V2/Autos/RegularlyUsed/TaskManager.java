package org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed;

import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.ForkTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.Task;
import com.roboctopi.cuttlefish.queue.TaskList;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

//Class that just maneges the tasks
public class TaskManager{
    TaskQueue queue;
    PTPController ptpController;
    PTPController otosController;
    PTPController fusionController;
    Battery battery;

    public TaskManager(TaskQueue queue, PTPController ptpController) {
        this.queue = queue;
        this.ptpController = ptpController;
    }

    public TaskManager(TaskQueue queue, PTPController ptpController, PTPController ptpOtosController, PTPController ptpFusionController) {
        this.queue = queue;
        this.ptpController = ptpController;
        this.otosController = ptpOtosController;
        this.fusionController = ptpFusionController;
    }

    public void forkTask(Task task1, Task task2) {//fork tasks, will this work??
        //class ForkTask(val task1: Task, val task2: Task) : Task
        queue.addTask(new ForkTask(task1, task2));
    }

    //to add a task
    public void addTask(Task task) {//got lazy and didn't want to change the TeleOp class code
        queue.addTask(task);
    }

    //basic driving
    public void waypointTask(TaskList task, Pose pose) {task.addTask(new PointTask(new Waypoint(pose) , ptpController));}


    //Gets in other stuff too -- power, rSlop, tSlop, passthrough
    //if want to switch through oto, odo, and fusion
    public void waypointTask(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), ptpController));
    }
    public void waypointTaskOtos(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), otosController));
    }
    public void waypointTaskFusion(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        taskList.addTask(new PointTask(new Waypoint(pose, power, rSlop, tSlop, passthrough), fusionController));
    }

    public void waypointTaskWithBattery(TaskList taskList, Pose pose, double power, double rSlop, double tSlop, boolean passthrough) {
        taskList.addTask(new PointTask(new Waypoint(pose, (battery.optimalSpeed(power)), rSlop, tSlop, passthrough), ptpController));
    }

    public void delay(TaskList taskList, int delay) {taskList.addTask(new DelayTask(delay));}//To add a delay task

    public void task(TaskList taskList, Runnable action){//to add a bunch of tasks
        taskList.addTask(new CustomTask(() -> {
            action.run();
            return true;
        }));
    }
}
