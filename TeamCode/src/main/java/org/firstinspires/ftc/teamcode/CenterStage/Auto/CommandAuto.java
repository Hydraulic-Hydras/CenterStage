package org.firstinspires.ftc.teamcode.CenterStage.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.hydraulichydras.hydrauliclib.Command.Command;
import com.hydraulichydras.hydrauliclib.Command.CommandSequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class CommandAuto extends LinearOpMode {

    public static final Pose2d RR_START_POSE = new Pose2d(35, -64, Math.toRadians(90));
    private SampleMecanumDrive drive;
    private TrajectorySequence preload;
    private TrajectorySequence preloadToConeStack;
    private Command preloadtraj = () -> drive.followTrajectorySequence(preload);

   private Command preloadtoConeStack = () -> drive.followTrajectorySequence(preloadToConeStack);
   private CommandSequence drivetoPreloadCommandSeq = new CommandSequence()
            .addCommand(preloadtraj)
            .build();

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(RR_START_POSE);

        preload = drive.trajectorySequenceBuilder(RR_START_POSE)
                .splineTo(new Vector2d(36.67, -30.50), Math.toRadians(106.73))
                .splineTo(new Vector2d(31.17, -11.17), Math.toRadians(136.17))

                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .setReversed(true)
                .splineTo(new Vector2d(58.50, -12.30), Math.toRadians(-2.16))
                .waitSeconds(.5)

                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
