package org.firstinspires.ftc.teamcode.CenterStage.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStage.Hardware.DriveTrain;

@TeleOp (name = "Qualifier TeleOp")
public class QualifierMode extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain(this);

    @Override
    public void runOpMode() {
        driveTrain.init(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driveTrain.RobotCentric(gamepad1);

               // driveTrain.FieldCentric(gamepad1);
            }
        }
    }
}
