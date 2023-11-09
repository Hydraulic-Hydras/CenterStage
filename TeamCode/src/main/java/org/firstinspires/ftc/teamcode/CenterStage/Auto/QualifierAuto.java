package org.firstinspires.ftc.teamcode.CenterStage.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class QualifierAuto extends LinearOpMode {

    public enum Side {
        LEFT,
        RIGHT,
        CENTER,
        IDLE
    }

    public enum ParkState {
        DRIVE,
        PARK,
        STOPPED
    }

    SampleMecanumDrive drive;

    ParkState parkState = ParkState.DRIVE;
    Side side = Side.IDLE;

    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = AutoConstants.BR_START;
        drive.setPoseEstimate(startPose);

            TrajectorySequence RIGHT_Prop = drive.trajectorySequenceBuilder(startPose)

                    .setConstraints(AutoConstants.Vel0, AutoConstants.Accel0)
                    .setTurnConstraint(60, 60)

                    .strafeRight(10.7)

                    .lineTo(new Vector2d(-46.7, 35.3))

                    .setTurnConstraint(50, 50)
                    .turn(Math.PI / -2)
                    .resetTurnConstraint()

                    .lineTo(new Vector2d(-58.3, 35.3))
                    .waitSeconds(1)

                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(-40, 12, Math.toRadians(180)), Math.toRadians(0))
                    .lineTo(new Vector2d(24, 12))
                    .splineToConstantHeading(new Vector2d(50, 41), Math.toRadians(0))

                    .waitSeconds(5)

                    .build();

            TrajectorySequence CENTER_Prop = drive.trajectorySequenceBuilder(startPose)

                    .setConstraints(AutoConstants.Vel0, AutoConstants.Accel0)
                    .setTurnConstraint(60, 60)

                    .lineTo(new Vector2d(-36, 24))

                    .lineToLinearHeading(new Pose2d(-59, 24, Math.toRadians(180)))
                    .waitSeconds(1)
                    .setReversed(true)

                    .splineToSplineHeading(new Pose2d(-40, 12, Math.toRadians(180)), Math.toRadians(0))

                    .lineTo(new Vector2d(24, 12))

                    // .lineToLinearHeading(new Pose2d(24, 12, Math.toRadians(0)))

                    //  .splineTo(new Vector2d(50, 35), Math.toRadians(0)) makes movement slower

                    .splineToConstantHeading(new Vector2d(50, 35), Math.toRadians(0))
                    .waitSeconds(5)

                    .build();

            TrajectorySequence LEFT_Prop = drive.trajectorySequenceBuilder(startPose)

                    .setConstraints(AutoConstants.Vel0, AutoConstants.Accel0)
                    .setTurnConstraint(60, 60)

                    .build();

            while (opModeIsActive() && !isStopRequested()) {
                drive.update();
                telemetry.update();

                switch (side) {
                    case LEFT:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequence(LEFT_Prop);
                            telemetry.addLine("Left Side");
                            telemetry.update();
                        }
                        parkState = ParkState.PARK;
                        time.reset();
                    case CENTER:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequence(CENTER_Prop);
                            telemetry.addLine("Center Side");
                            telemetry.update();
                        }
                        parkState = ParkState.PARK;
                        time.reset();
                    case RIGHT:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequence(RIGHT_Prop);
                            telemetry.addLine("Right Side");
                            telemetry.update();
                        }
                        parkState = ParkState.PARK;
                        time.reset();
                        break;
                    case IDLE:
                        if (time.seconds() > 1.5) {
                            telemetry.addLine("Auto finished");
                            stop();
                            parkState = ParkState.STOPPED;

                        }
                        break;
                }
            }
    }
}