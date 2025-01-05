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
    public static boolean test;
    public static String color;
    public static String side;
    TaskManager manager;
    TeleOp teleOp;

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

        manager = new TaskManager(queue, ptpController);
        teleOp = new TeleOp(intake,outake, extendo, lift, dt, manager);
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

    //prints out all data, odometry and otos
    public void allLocationData() {
        otosLocalizer.update();
        encoderLocalizer.update();
        Pose otos = otosLocalizer.getPos();
        Pose odo = encoderLocalizer.getPos();

        telemetry.addData("Oto X:", otos.getX());
        telemetry.addData("Oto Y:", otos.getY());
        telemetry.addData("Oto R:", otos.getR());

        telemetry.addData("Odo X:", odo.getX());
        telemetry.addData("Odo Y:", odo.getY());
        telemetry.addData("Odo R:", odo.getR());
        telemetry.update();
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
}
