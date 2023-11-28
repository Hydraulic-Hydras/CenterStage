package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Path.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Hardware.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.ThreeWheelOdom;
import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Util.HSubsystem;
import org.firstinspires.ftc.teamcode.common.commands.PositionCommand;


@Autonomous
public class AutoTest extends CommandOpMode {

    public final RobotHardware robot = RobotHardware.getInstance();
    private HSubsystem mecanumDrive;
    private ThreeWheelOdom localizer;
    public double looptime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        // add mecanum drive train code here
        mecanumDrive = new MecanumDrive();
        localizer = new ThreeWheelOdom();

        robot.addSubsystem(mecanumDrive);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("Robot in init");
            telemetry.update();
        }

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Pose test = new Pose(0,10,Math.PI/2);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // initial test
                      new PositionCommand((Drivetrain) mecanumDrive, localizer, test)
                )
        );
    }

    @Override
    public void run() {
        robot.read();

        super.run();
        robot.periodic();
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / ( loop - looptime));
        telemetry.addData("voltage", robot.getVoltage());
        looptime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }

}
