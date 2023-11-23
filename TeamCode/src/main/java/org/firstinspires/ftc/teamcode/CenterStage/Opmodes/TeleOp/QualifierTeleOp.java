package org.firstinspires.ftc.teamcode.CenterStage.Opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStage.common.Hardware.Robot;

@TeleOp (name = "Qualifier TeleOp")
public class QualifierTeleOp extends LinearOpMode {

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        robot.initialize(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

            robot.loop(gamepad1, gamepad2);
            robot.telemetry(telemetry);

            }
        }
    }
}
