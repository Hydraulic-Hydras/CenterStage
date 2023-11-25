package org.firstinspires.ftc.teamcode.CenterStage.Opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStage.common.Hardware.RobotHardware;

@TeleOp
public class QualifierTeleOp extends CommandOpMode {

   private final RobotHardware robot = RobotHardware.getInstance();
    // public MecanumDrive mecanumDrive;
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        // robot.init(hardwareMap, telemetry);
       // mecanumDrive = new MecanumDrive();
       // mecanumDrive.RobotCentric(gamepad1);

        // robot.addSubsystem(mecanumDrive);

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized. Mr WorldWide we going to the TOPPPPPPPPPPPP, DALEE");
            telemetry.addLine("Also hi Mr Bao bun");
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
