package org.firstinspires.ftc.teamcode.opmodes.Auto.Advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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


@Autonomous (name = "ADV_Red Right")
public class Adv_RedRightBK extends CommandOpMode {

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
    public static TrajectorySequence preloadLeft;
    public static TrajectorySequence preloadRight;

    public static TrajectorySequence BackdropLeft;
    public static TrajectorySequence BackdropCenter;
    public static TrajectorySequence BackdropRight;
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

        // Poses
        Pose2d startPose = Globals.StartPoseRED_RIGHT;
        Pose2d backdrop_pos = Globals.RED_SPIKEMARK_RIGHT;

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
                .lineToLinearHeading(Globals.RED_SPIKEMARK_RIGHT)

                .build();

        // LEFT PROP
        preloadLeft = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .lineToLinearHeading(Globals.RED_TEAMPROP_LEFT_POSE)

                .build();

        // RIGHT PROP
        preloadRight = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                // spline movement towards it
                .splineToConstantHeading(new Vector2d(23, -40), Math.toRadians(90))

                .build();


        // PARK
        park = drive.trajectorySequenceBuilder(backdrop_pos)
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

                                new InstantCommand(() -> drive.followTrajectorySequence(Backdrop))
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

                                new InstantCommand(() -> drive.followTrajectorySequence(Backdrop))
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

                                new InstantCommand(() -> drive.followTrajectorySequence(Backdrop))
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

