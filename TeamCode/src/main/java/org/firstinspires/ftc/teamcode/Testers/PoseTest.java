package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Hardware.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.ThreeWheelOdom;
import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Util.HSubsystem;

@TeleOp
public class PoseTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private HSubsystem drivetrain;
    private ThreeWheelOdom localizer;

    private boolean started;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap,telemetry);
        localizer = new ThreeWheelOdom();
        drivetrain = new MecanumDrive();

        robot.addSubsystem(drivetrain);

        robot.enabled = true;

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("Auto init");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        if (!started) {
            started = true;
            localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        }
        robot.clearBulkCache();

        localizer.periodic();
        super.run();

        Pose currentPose = localizer.getPos();
        telemetry.addData("poseX", currentPose.x);
        telemetry.addData("poseY", currentPose.y);
        telemetry.addData("heading", currentPose.heading);
        // TODO: Add telemetry for encoders

        telemetry.update();
    }
}
