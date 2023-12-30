package org.firstinspires.ftc.teamcode.opmodes.Auto.Basic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.LiftCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.Hardware.Constants.Globals;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Vision.PropVision;
import org.firstinspires.ftc.teamcode.common.Hardware.Vision.TFOD;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "BASIC_Red Right")
public class Bsc_RedRightBK extends CommandOpMode {

    // Contraptions
    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    private final TFOD tfod = new TFOD(this);

    // Drivetrain
    private SampleMecanumDrive drive;

    // Vision
    private final PropVision propVision = new PropVision(telemetry);
    private Side location;

    // Timer
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    // Trajectories
    public static TrajectorySequence preloadCenter;
    public static TrajectorySequence turningCenter;

    public static TrajectorySequence preloadLeft;
    public static TrajectorySequence turningLeft;

    public static TrajectorySequence preloadRight;
    public static TrajectorySequence turningRight;

    public static TrajectorySequence park;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        drive = new SampleMecanumDrive(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        intake.initialize(hardwareMap);
        tfod.initialize(hardwareMap);

        tfod.addProcessor(propVision);

        location = PropVision.getSide();

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

        while (!isStarted()) {

            telemetry.addLine("Auto in init");
            telemetry.addLine();

            telemetry.addData("X", startPose.getX());
            telemetry.addData("Y", startPose.getY());
            telemetry.addData("Heading", startPose.getHeading());

            telemetry.addData("Prop Location", location);
            telemetry.addLine();

            telemetry.update();
        }

        telemetry.addData("Prop Location", location);
        tfod.close();

        // CENTER PROP
        preloadCenter = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(29)

                .build();

        turningCenter = drive.trajectorySequenceBuilder(preloadCenter.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .back(5)
                .turn(Math.toRadians(90))
                .back(33.8)
                .strafeRight(3.2)

                .build();

        // LEFT PROP
        preloadLeft = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(29)
                .turn(Math.toRadians(90))

                .build();

        turningLeft = drive.trajectorySequenceBuilder(preloadLeft.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .back(33.8)
                .strafeRight(6)

                .build();

        // RIGHT PROP
        preloadRight = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(29)
                .turn(Math.toRadians(-90))

                .build();

        turningRight = drive.trajectorySequenceBuilder(preloadRight.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(33.8)
                .turn(Math.toRadians(180))
                .strafeLeft(8)

                .build();

        // LEFT Park
        park = drive.trajectorySequenceBuilder(new Pose2d())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .strafeLeft(15)
                .back(5)

                .build();

    }

    @Override
    public void run() {

        switch (location) {
            case CENTER:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(timer::reset),

                                new InstantCommand(() -> drive.followTrajectorySequence(preloadCenter))
                                        .alongWith(new IntakeReverseCommand()),

                                new WaitCommand(700),
                                new IntakeStopCommand(),

                                new InstantCommand(() -> drive.followTrajectorySequence(turningCenter))
                                        .alongWith(new LiftCommand(1250, 1)),

                                new WaitCommand(900),
                                new OuttakeCommand(),

                                new LiftCommand(100, 0.65)
                                        .alongWith(new InstantCommand(() -> drive.followTrajectorySequence(park))),

                                new InstantCommand(() -> endTime = timer.seconds())
                        )
                );

                break;

            case LEFT:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(timer::reset),

                                new InstantCommand(() -> drive.followTrajectorySequence(preloadLeft))
                                        .alongWith(new IntakeReverseCommand()),

                                new WaitCommand(700),
                                new IntakeStopCommand(),

                                new InstantCommand(() -> drive.followTrajectorySequence(turningLeft))
                                        .alongWith(new LiftCommand(1500, 1)),

                                new WaitCommand(900),
                                new OuttakeCommand(),

                                new LiftCommand(100, 0.65)
                                        .alongWith(new InstantCommand(() -> drive.followTrajectorySequence(park))),

                                new InstantCommand(() -> endTime = timer.seconds())

                        )
                );

                break;

            case RIGHT:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(timer::reset),

                                new InstantCommand(() -> drive.followTrajectorySequence(preloadRight))
                                        .alongWith(new IntakeReverseCommand()),

                                new WaitCommand(700),
                                new IntakeStopCommand(),

                                new InstantCommand(() -> drive.followTrajectorySequence(turningRight))
                                        .alongWith(new LiftCommand(1500, 1)),
                                new WaitCommand(900),
                                new OuttakeCommand(),

                                 new LiftCommand(100, 0.65)
                                        .alongWith(new InstantCommand(() -> drive.followTrajectorySequence(park))),

                                new InstantCommand(() -> endTime = timer.seconds())

                        )
                );
                break;
        }

        // Run OpMode
        super.run();

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        telemetry.addLine();

        // localizer
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

    }
}

