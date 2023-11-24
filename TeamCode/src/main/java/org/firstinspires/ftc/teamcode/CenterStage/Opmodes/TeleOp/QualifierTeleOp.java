package org.firstinspires.ftc.teamcode.CenterStage.Opmodes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStage.common.Hardware.Contraptions.DriveTrain;
import org.firstinspires.ftc.teamcode.CenterStage.common.Hardware.RobotHardware;

@TeleOp
public class QualifierTeleOp extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    public DriveTrain driveTrain;
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        driveTrain = new DriveTrain();

        driveTrain.RobotCentric(gamepad1);
        robot.addSubsystem(driveTrain);

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();

        // input
        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        robot.write();
        robot.clearBulkCache();
    }
}