package org.firstinspires.ftc.teamcode.CenterStage.Auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStage.PipeLines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous (name = "FSM Custom Park")
public class DriveTrain_FSM extends LinearOpMode {

    public enum parkState {
        DRIVE,
        PARK,
        IDLE
    }

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    final int left = 1;
    final int middle = 2;
    final int right = 3;

    ElapsedTime time = new ElapsedTime();

    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive;

    // speed variables
    private static final TrajectoryVelocityConstraint FAST_VELO = SampleMecanumDrive.getVelocityConstraint(100, Math.toRadians(180), Math.toRadians(180));
    private static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(100);
    private static final TrajectoryVelocityConstraint VEL = SampleMecanumDrive.getVelocityConstraint(45, Math.toDegrees(180), Math.toRadians(180));
    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(45);

    parkState drivetrain = parkState.DRIVE;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // starting pose
        // blue left
        Pose2d startpose = new Pose2d(-35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startpose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        telemetry.setMsTransmissionInterval(50);

        TrajectorySequence pos = drive.trajectorySequenceBuilder(startpose)
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .forward(45)

                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(pos.end())
                .setConstraints(VEL, ACCEL)
                .strafeLeft(10)
                //.lineToLinearHeading(AutoConstants.BL_LEFT_PARK)
                .build();

        TrajectorySequence toRightPark = drive.trajectorySequenceBuilder(pos.end())
                .setConstraints(VEL, ACCEL)
                .strafeRight(10)
                //.lineToLinearHeading(AutoConstants.BL_RIGHT_PARK)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(pos.end())
                .setConstraints(VEL, ACCEL)
                .back(10)
                //.lineToLinearHeading(AutoConstants.BL_MIDDLE_PARK)
                .build();


        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            telemetry.update();

            switch (drivetrain) {
                case DRIVE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(pos);
                        drivetrain = parkState.PARK;
                    }
                case PARK:
                    if (tagOfInterest == null || tagOfInterest.id == right) {
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(toRightPark);
                            drivetrain = parkState.IDLE;
                            if (tagOfInterest == null || tagOfInterest.id == right) {
                                //middle code
                                telemetry.addLine("right");
                                telemetry.update();
                            }
                            time.reset();
                        }
                    } else if (tagOfInterest.id == middle) {
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(toMiddlePark);
                            drivetrain = parkState.IDLE;
                            if (tagOfInterest == null || tagOfInterest.id == middle) {
                                //middle code
                                telemetry.addLine("middle");
                                telemetry.update();
                            }
                            time.reset();
                        }
                    } else if (tagOfInterest.id == left) {
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(toLeftPark);
                            drivetrain = parkState.IDLE;
                            if (tagOfInterest == null || tagOfInterest.id == left) {
                                //middle code
                                telemetry.addLine("left");
                                telemetry.update();
                            }
                            time.reset();
                        }
                    }
                    break;
                case IDLE:
                    if (time.seconds() > 1.5) {
                        if (!drive.isBusy()) {
                            telemetry.addLine("Park finished");
                            stop();
                        }
                    }
                    break;
            }


            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry  */
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry (AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        // telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        // telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        // telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.update();
    }
}