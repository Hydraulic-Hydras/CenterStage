package org.firstinspires.ftc.teamcode.CenterStage.Opmodes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Path.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CenterStage.common.Hardware.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.CenterStage.common.Hardware.Drive.ThreeWheelOdom;
import org.firstinspires.ftc.teamcode.CenterStage.common.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.CenterStage.common.Util.HSubsystem;
import org.firstinspires.ftc.teamcode.CenterStage.common.commands.PositionCommand;

@Config
@Autonomous
public class RedAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private ThreeWheelOdom localizer;
    private HSubsystem drivetrain;

    private double loopTime = 0.0;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        drivetrain = new MecanumDrive();
        localizer = new ThreeWheelOdom();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.addSubsystem(drivetrain);
        robot.read();

        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Pose yellowScorePos = new Pose(21, 26.15, -1.5);
        Pose purpleScorePos = new Pose(28, 25, -1.5);
        Pose parkPos = new Pose(6, 31, -3 * Math.PI / 2);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // scoring position
                        new PositionCommand((Drivetrain) drivetrain, localizer, yellowScorePos),
                        new WaitCommand(1000),
                        new PositionCommand((Drivetrain) drivetrain, localizer, purpleScorePos),
                        new WaitCommand(1000),
                        new PositionCommand((Drivetrain) drivetrain, localizer, parkPos)
                )
        );
    }

        @Override
        public void run () {
            robot.read();

            super.run();
            robot.periodic();
            localizer.periodic();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();

            robot.write();
            robot.clearBulkCache();
        }
    }
