package org.firstinspires.ftc.teamcode.Testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;

@TeleOp
public class CommandOpModeVisionTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private double loopTime = 0.0;
    @Override
    public void initialize() {
        robot.init(hardwareMap, telemetry);

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot initialized");
            robot.tensorFlow.Tfod_location(telemetry);
            telemetry.update();
        }

    }

    @Override
    public void run() {
        robot.read();


        super.run();
        robot.periodic();
        double loop = System.nanoTime();

        telemetry.addData("hz", 1000000000 / (loop - loopTime));
        loopTime = loop;

        telemetry.update();
        robot.write();
        robot.clearBulkCache();
    }
}


