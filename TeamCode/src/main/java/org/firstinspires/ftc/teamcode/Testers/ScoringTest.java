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
import org.firstinspires.ftc.teamcode.common.Commands.Auto.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.LiftCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.OuttakeCommand;
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

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        drive = new SampleMecanumDrive(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();
        intake.initialize(hardwareMap);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)

                .forward(10)
                .build();

        Intake.rotateBucket.setPosition(Intake.POS_REST);

        while (!isStarted()) {
            telemetry.addLine("Auto in init");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new InstantCommand(() -> drive.followTrajectorySequence(test)),
                        new WaitCommand(100),
                        new IntakeCommand(),
                        new LiftCommand(1500, 0.75),
                        new OuttakeCommand(),

                        new LiftCommand(0, 0.5)

                ));


    }

    @Override
    public void run() {
        super.run();

        telemetry.update();
    }
}
