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
import org.firstinspires.ftc.teamcode.common.CommandBase.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.common.CommandBase.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.CommandBase.LiftCommand;
import org.firstinspires.ftc.teamcode.common.CommandBase.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "JAVA-ScoringTest")
public class ScoringTest extends CommandOpMode {

    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    private SampleMecanumDrive drive;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        drive = new SampleMecanumDrive(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        intake.initialize(hardwareMap);

        while (!isStarted()) {

            telemetry.addLine("Auto in init");
            telemetry.addLine();

            telemetry.update();
        }

        TrajectorySequence preload = drive.trajectorySequenceBuilder(new Pose2d())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(29)

                .build();

        TrajectorySequence turning = drive.trajectorySequenceBuilder(preload.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .back(5)
                .turn(Math.toRadians(90))

                .build();

        TrajectorySequence toBK = drive.trajectorySequenceBuilder(turning.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .back(30.5)

                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toBK.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .strafeLeft(16)
                .back(9)

                .build();


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new InstantCommand(() -> drive.followTrajectorySequence(preload)),

                        new WaitCommand(100),
                        new IntakeReverseCommand(),

                        new WaitCommand(700),
                        new IntakeStopCommand(),

                        new WaitCommand(100),
                        new InstantCommand(() -> drive.followTrajectorySequence(turning)),

                        new InstantCommand(() -> drive.followTrajectorySequence(toBK)),
                        new LiftCommand(1450, 1),

                        new WaitCommand(900),
                        new OuttakeCommand(),

                        new WaitCommand(700),

                        new InstantCommand(() -> drive.followTrajectorySequence(park))
                                .alongWith(new LiftCommand(0, 0.65))


                ));

    }

    @Override
    public void run() {
        super.run();


    }

}
