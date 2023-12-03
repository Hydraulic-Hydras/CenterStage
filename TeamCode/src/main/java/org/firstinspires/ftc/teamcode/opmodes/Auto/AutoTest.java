package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.hydraulichydras.hydrauliclib.Geometry.Point;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;

import com.hydraulichydras.hydrauliclib.Path.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Hardware.Constants.PoseStorage;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.ThreeWheelOdom;
import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Pathing.ChaiTrackingPath;
import org.firstinspires.ftc.teamcode.common.Pathing.WayPoint;
import org.firstinspires.ftc.teamcode.common.Util.HSubsystem;
import org.firstinspires.ftc.teamcode.common.commands.ChaiTrackingCommand;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.nio.file.Watchable;

@Disabled
@Autonomous
public class AutoTest extends CommandOpMode {

    public final RobotHardware robot = RobotHardware.getInstance();
    private HSubsystem drivetrain;
   //  private ThreeWheelOdom localizer;
    private double loopTime = 0.0;

    private Pose[] STRAIGHT_TEST = new Pose[]{
            new Pose(0, 10, 0),
            new Pose(0, 20, 0)
    };

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        drivetrain = new MecanumDrive();
        // localizer = new ThreeWheelOdom();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.addSubsystem(drivetrain);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

       // localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        ChaiTrackingPath movement1 = new ChaiTrackingPath(
                new WayPoint(STRAIGHT_TEST[1], 20),
                new WayPoint(STRAIGHT_TEST[2], 20)
        );


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // new ChaiTrackingCommand((Drivetrain) drivetrain, localizer, movement1),
                        new WaitCommand(200)
                ));
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        robot.read();

        CommandScheduler.getInstance().run();
        robot.periodic();
       //  localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        // telemetry.addData("pose", localizer.getPos());
        telemetry.addData("voltage", robot.getVoltage());
        loopTime = loop;
        telemetry.update();
        // hopefully pose transfer works
        // PoseStorage.currentPose = localizer.getPoseEstimate();

        robot.write();
    }
}
