package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class PositionTest extends CommandOpMode {

    private SampleMecanumDrive drive;
    private ElapsedTime timer = new ElapsedTime();
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

        TrajectorySequence goForward = drive.trajectorySequenceBuilder(startPose)

                .forward(20)

                .build();

        while (!isStarted()) {
            telemetry.addLine("Auto in init");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer:: reset),

                        new InstantCommand(() -> drive.followTrajectorySequence(goForward)),
                        new WaitCommand(1000)
                )
        );

    }


    @Override
    public void run() {
        super.run();
        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}
