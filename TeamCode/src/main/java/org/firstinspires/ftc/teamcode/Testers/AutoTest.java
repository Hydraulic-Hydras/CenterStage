package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.common.Hardware.Robot;

@Autonomous
public class AutoTest extends LinearOpMode {
    private final Robot robot = new Robot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        robot.drive.setPoseEstimate(Globals.StartPose);

    }
}
