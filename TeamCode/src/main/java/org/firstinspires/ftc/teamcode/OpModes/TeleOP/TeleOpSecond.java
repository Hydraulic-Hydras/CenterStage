package org.firstinspires.ftc.teamcode.OpModes.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;

@TeleOp (name = "Java TeleOp, Using Sensors")
public class TeleOpSecond extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                robot.loop(gamepad1, gamepad2);
            }
        }
    }
}
