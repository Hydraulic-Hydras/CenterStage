package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.LiftCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.Hardware.Constants.Globals;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous (name = "JAVA-ScoringTest")
public class ScoringTest extends CommandOpMode {

    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    private SampleMecanumDrive drive;

    private final ElapsedTime timer = new ElapsedTime();

    private double endTime = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        drive = new SampleMecanumDrive(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        intake.initialize(hardwareMap);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

        while (!isStarted()) {

            telemetry.addLine("Auto in init");
            telemetry.addLine();

            telemetry.addData("X", startPose.getX());
            telemetry.addData("Y", startPose.getY());
            telemetry.addData("Heading", startPose.getHeading());

            telemetry.update();
        }

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)

                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .build();

        TrajectorySequence turning = drive.trajectorySequenceBuilder(preload.end())

                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .back(5)
                .turn(Math.toRadians(90))
                .back(33.8)
                .strafeRight(3.2)

                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(turning.end())

                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .strafeLeft(15)
                .back(5)

                .build();


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new InstantCommand(() -> drive.followTrajectorySequence(preload))
                                .alongWith(new IntakeReverseCommand()),

                        new WaitCommand(700),
                        new IntakeStopCommand(),

                        new InstantCommand(() -> drive.followTrajectorySequence(turning))
                                .alongWith(new LiftCommand(1250, 1)),

                        new WaitCommand(900),
                        new OuttakeCommand(),

                        new LiftCommand(100, 0.65)
                                .alongWith(new InstantCommand(() -> drive.followTrajectorySequence(park))),

                        new InstantCommand(() -> endTime = timer.seconds())


                ));


    }

    @Override
    public void run() {
        super.run();

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        telemetry.addLine();

        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

}
