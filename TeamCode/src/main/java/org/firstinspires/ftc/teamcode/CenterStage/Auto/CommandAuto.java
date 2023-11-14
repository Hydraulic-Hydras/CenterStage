package org.firstinspires.ftc.teamcode.CenterStage.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.hydraulichydras.hydrauliclib.Command.AutoCommandMachine;
import com.hydraulichydras.hydrauliclib.Command.Command;
import com.hydraulichydras.hydrauliclib.Command.CommandSequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class CommandAuto extends LinearOpMode {

    public static final Pose2d RR_START_POSE = new Pose2d(35, -64, Math.toRadians(0));
    private SampleMecanumDrive drive;
    private TrajectorySequence preload;
    private TrajectorySequence preloadToConeStack;
    private Command preloadtraj = () -> drive.followTrajectorySequence(preload);

   private Command preloadtoConeStack = () -> drive.followTrajectorySequence(preloadToConeStack);
   private CommandSequence drivetoPreloadCommandSeq = new CommandSequence()
            .addCommand(preloadtraj)
            .build();

    private AutoCommandMachine autoCommandMachine = new AutoCommandMachine()
            .addCommandSequence(drivetoPreloadCommandSeq)
            .build();

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(RR_START_POSE);

        preload = drive.trajectorySequenceBuilder(RR_START_POSE)
                .forward(50)
                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .turn(Math.toRadians(0))

                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            autoCommandMachine.run(drive.isBusy());

        }
    }
}
