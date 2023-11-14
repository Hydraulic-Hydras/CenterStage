package org.firstinspires.ftc.teamcode.CenterStage.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    DistanceSensor backSense;

    boolean FAILSAFE_ACTIVATED = false;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        backSense = hardwareMap.get(DistanceSensor.class, "Distance_Sensor");

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
                .lineTo(RedConstants.CENTER_DOOR_DELAYPOS_VECTOR, RedConstants.SlowVel, RedConstants.SlowAccel)

                .waitSeconds(2)

                .build();

        TrajectorySequence failSafe = drive.trajectorySequenceBuilder(new Pose2d())

                .setConstraints(RedConstants.Vel0, RedConstants.Accel0)
                .strafeLeft(24)

                .resetConstraints()
                .back(20)

                .build();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", heading);
            telemetry.update();

            switch (drivetrain) {
                case PRELOAD:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequence(preload);

                            drivetrain = driveState.STACK;
                            sequences = AutoSequences.WORKING;

                            time.reset();
                        }
                case STACK:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequence(whitePOWER);

                            drivetrain = driveState.DOOR;

                            time.reset();
                        }

                case DOOR:
                    if (!drive.isBusy()) {
                        // if loop if the distance sensor is being used
                        if (backSense.getDistance(DistanceUnit.INCH) == 15 ) {
                            drive.breakFollowing();
                            sequences = AutoSequences.FAILSAFE;
                            FAILSAFE_ACTIVATED = true;
                            // Stop the motors
                            drive.setDrivePower(new Pose2d());
                        }   else {
                            drive.followTrajectorySequence(doorPos);
                        }

                        // if loop if the robot collides and crashes
                        if (heading > 185 || heading < 175 && time.seconds() >= 3) {
                            // Cancel Following
                            drive.breakFollowing();
                            sequences = AutoSequences.FAILSAFE;
                            FAILSAFE_ACTIVATED = true;
                            // Stop the motors
                            drive.setDrivePower(new Pose2d());
                        }   else {
                            drive.followTrajectorySequence(doorPos);
                        }

                        time.reset();
                    }

                    break;
                        case IDLE:
                            if (time.seconds() > 1.5) {
                                if (!drive.isBusy()) {
                                    sequences = AutoSequences.STOP;
                                }
                            }
                        }

                        switch (sequences) {
                            case WORKING:
                                // do nothing basically other than display information
                                drive.update();

                            case FAILSAFE:
                                sleep(1000);
                                if (!drive.isBusy() && FAILSAFE_ACTIVATED) {
                                    drive.followTrajectorySequence(failSafe);
                                }

                                drivetrain = driveState.IDLE;

                            case STOP:
                                stop();
                    }
        }
    }
}

