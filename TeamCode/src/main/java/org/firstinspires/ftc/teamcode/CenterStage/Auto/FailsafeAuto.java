package org.firstinspires.ftc.teamcode.CenterStage.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStage.RedConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class FailsafeAuto extends LinearOpMode {
    public enum driveState {
        PRELOAD,
        STACK,
        DOOR,
        IDLE
    }

    driveState drivetrain = driveState.PRELOAD;

    public enum AutoSequences {
        WORKING,
        FAILSAFE,
        STOP
    }

    ElapsedTime time = new ElapsedTime();
    SampleMecanumDrive drive;
    AutoSequences sequences = AutoSequences.WORKING;
    Telemetry telemetry;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        // Right left
        drive.setPoseEstimate(RedConstants.LEFT_SP);
        Pose2d poseEstimate = drive.getPoseEstimate();

        double x = poseEstimate.getX();
        double y = poseEstimate.getY();
        double heading = poseEstimate.getHeading();

        TrajectorySequence preload = drive.trajectorySequenceBuilder(RedConstants.LEFT_SP)
                .setConstraints(RedConstants.Vel0, RedConstants.Accel0)

                // left prop test
                .lineToLinearHeading(RedConstants.L_LEFT_PROP_UNLOAD_POSE)
                .waitSeconds(1)

                .build();

        TrajectorySequence whitePOWER = drive.trajectorySequenceBuilder(preload.end())

                .strafeRight(1)
                .splineToConstantHeading(RedConstants.UPPER_WHITE_STACK_VECTOR, RedConstants.HEADING_HIGH_PIXEL)
                .waitSeconds(1)

                .build();

        TrajectorySequence doorPos = drive.trajectorySequenceBuilder(whitePOWER.end())

                .setReversed(true)
                .addDisplacementMarker(15, () -> {
                    telemetry.addData("x", x);
                    telemetry.addData("y", y);
                    telemetry.addData("heading", heading);
                    telemetry.update();
                })

                .lineTo(RedConstants.CENTER_DOOR_DELAYPOS_VECTOR)
                .waitSeconds(2)

                .build();

        TrajectorySequence failSafe = drive.trajectorySequenceBuilder(new Pose2d())

                .addTemporalMarker(0, () -> {
                    telemetry.addData("heading", heading);
                    telemetry.update();

                })
                .build();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            telemetry.update();

            switch (drivetrain) {
                case PRELOAD:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequence(preload);

                            drivetrain = driveState.STACK;
                            sequences = AutoSequences.WORKING;

                            telemetry.addData("x", poseEstimate.getX());
                            telemetry.addData("y", poseEstimate.getY());
                            telemetry.addData("heading", poseEstimate.getHeading());

                            time.reset();
                        }
                case STACK:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequence(whitePOWER);

                            drivetrain = driveState.DOOR;
                            sequences = AutoSequences.WORKING;

                            time.reset();
                        }


                case DOOR:
                    if (!drive.isBusy()) {
                            drive.followTrajectorySequence(doorPos);
                            sequences = AutoSequences.WORKING;

                        if (heading > 0) {
                            drive.followTrajectorySequence(failSafe);
                            sequences = AutoSequences.FAILSAFE;
                        } else {
                            sleep(2000);
                        }

                        time.reset();
                    }

                    break;
                        case IDLE:
                            if (time.seconds() > 1.5) {
                                if (!drive.isBusy()) {
                                    stop();
                                }
                            }
                        }

                        // TODO: fill in for these

                        switch (sequences) {
                            case WORKING:

                            case FAILSAFE:

                            case STOP:
                    }
        }
    }
}

