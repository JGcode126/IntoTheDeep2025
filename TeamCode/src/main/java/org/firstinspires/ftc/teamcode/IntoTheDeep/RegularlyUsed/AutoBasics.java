package org.firstinspires.ftc.teamcode.IntoTheDeep.RegularlyUsed;

import com.roboctopi.cuttlefish.controller.MotorPositionController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskQueue;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Init.CuttleInitOpMode;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleDT;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleExtendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleIntake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleOutake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Subsystems.CuttleSlides;

public class AutoBasics extends Setup {

    public AutoBasics(ThreeEncoderLocalizer otos, ThreeEncoderLocalizer encoderLocalizer, CuttleIntake intake, CuttleOutake outake, Telemetry telemetry, TaskQueue queue, PTPController ptpController, MotorPositionController liftController, MotorPositionController extController, CuttleExtendo extendo, CuttleSlides lift, CuttleDT dt) {
        super(otos, encoderLocalizer, intake, outake, telemetry, queue, ptpController, liftController, extController, extendo, lift, dt);
    }


}
