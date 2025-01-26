package org.firstinspires.ftc.teamcode.Robot_V2.Autos.RegularlyUsed;

import org.firstinspires.ftc.teamcode.Robot_V2.Init.CuttleInitOpModeRobot2;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleDT;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleExtendo;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleIntake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleOutake;
import org.firstinspires.ftc.teamcode.Robot_V2.Subsystems.v2CuttleSlides;

public class TeleOp extends CuttleInitOpModeRobot2 {
    public double highChamberPos = 5;
    public double highBucketPos = 14;

    v2CuttleIntake intake;
    v2CuttleSlides lift;
    v2CuttleOutake outake;
    v2CuttleExtendo extendo;
    v2CuttleDT dt;
    TaskManager manager;

    public TeleOp(v2CuttleIntake intake, v2CuttleOutake outake, v2CuttleExtendo extendo,
                  v2CuttleSlides lift, v2CuttleDT dt, TaskManager manager) {

        this.intake = intake;
        this.lift = lift;
        this.outake = outake;
        this.extendo = extendo;
        this.dt = dt;
        this.manager = manager;

    }

}
