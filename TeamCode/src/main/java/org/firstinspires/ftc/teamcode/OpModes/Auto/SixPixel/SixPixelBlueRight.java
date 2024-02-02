package org.firstinspires.ftc.teamcode.OpModes.Auto.SixPixel;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.common.Hardware.LEDS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled
@Autonomous (name = "4 Pixel Blue Right APR", group = " 2 + 4 ")
public class SixPixelBlueRight extends LinearOpMode {

    // Hardware Setup
    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    private SampleMecanumDrive drive;

    // Led
    private final LEDS leds = new LEDS(this);

    // Vision
    public List<Recognition> myTfodRecognitions;
    public TfodProcessor myTfodProcessor;
    public Recognition myTfodRecognition;
    public VisionPortal Cam1Portal;
    public double propLocation;
    public boolean USE_WEBCAM;
    public double x;
    public float y;

    // AprilTags
    public boolean USE_TAGS;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal AprilTagPortal;

    public Runnable ScoringBackdrop = () -> {
        try {
            mitsumi.autoMoveTo(1250, 1);
            Intake.rotateBucket.setPosition(Intake.POS_PANEL);
            Thread.sleep(500);
            Intake.rotateBucket.setPosition(Intake.POS_DUMP);
            Thread.sleep(1000);
            Intake.rotateBucket.setPosition(Intake.POS_PANEL);
            Thread.sleep(100);
            Intake.rotateBucket.setPosition(Intake.POS_REST);
            mitsumi.autoMoveTo(-200, 0.75);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };
    public void runThread(Thread thread) {
        try {
            thread.start();
        } catch (IllegalThreadStateException ignored) {}
    }

    private enum AutoState {
        SPIKEMARK,
        ONE_STACK,
        BACKDROP_SCORE,
        SECOND_STACK,
        BACKDROP_SECOND_SCORE,
        PARK,
        IDLE
    }

    public static double PARK;
    // ** Useful **
    public Side side = Side.LEFT;
    AutoState autoState = AutoState.SPIKEMARK;
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = true;

        // Initialize hardware
        drive = new SampleMecanumDrive(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();
        intake.initialize(hardwareMap);
        leds.initialize(hardwareMap);

        USE_WEBCAM = true;
        initTfod();

        while (!isStarted()) {
            propLocation = scanLocation();
            telemetry.addData("Team Prop Location", propLocation);
            telemetry.addData("Side: ", getSide());
            telemetry.update();
        }

        Thread BackdropScoreThread = new Thread(ScoringBackdrop);

        drive.setPoseEstimate(Globals.StartPose);

        TrajectorySequence spikemarkRight = drive.trajectorySequenceBuilder(Globals.StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .turn(Math.toRadians(-90))
                .forward(4)
                .addTemporalMarker(Intake::reverseIntake)
                .back(4)
                .addTemporalMarker(Intake::stopIntaking)
                .strafeLeft(29)
                .forward(21)

                .build();

        TrajectorySequence spikemarkLeft = drive.trajectorySequenceBuilder(Globals.StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .turn(Math.toRadians(90))
                .forward(4)
                .addTemporalMarker(Intake::reverseIntake)
                .back(10)
                .addTemporalMarker(Intake::stopIntaking)

                .setTurnConstraint(360, 360)
                .turn(Math.toRadians(180))
                .resetTurnConstraint()

                .splineToConstantHeading(new Vector2d(58, -18), Math.toRadians(-90))

                .build();

        TrajectorySequence spikemarkCenter = drive.trajectorySequenceBuilder(Globals.StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .addTemporalMarker(Intake::reverseIntake)
                .back(4)
                .strafeRight(16.5)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(Intake::stopIntaking)

                .splineToConstantHeading(new Vector2d(58, -18), Math.toRadians(-90))
                .forward(4)

                .build();

        TrajectorySequence One_stack = drive.trajectorySequenceBuilder(spikemarkRight.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .addTemporalMarker(Intake::fingerDown)
                .waitSeconds(1)
                .back(8)
                .addTemporalMarker(Intake::fingerReset)
                .forward(9.5)
                .addTemporalMarker(Intake::startIntaking)
                .waitSeconds(1) // changed from 0.5 to 1
                .back(20)
                .addTemporalMarker(Intake::stopIntaking)

                .build();

        TrajectorySequence Backdrop_score = drive.trajectorySequenceBuilder(One_stack.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .back(65)
                .splineToConstantHeading(new Vector2d(26, 74.5), Math.toRadians(-90))
                .back(1)

                .build();

        TrajectorySequence Secondstack = drive.trajectorySequenceBuilder(Backdrop_score.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .build();

        TrajectorySequence backdropScoreTwo = drive.trajectorySequenceBuilder(Secondstack.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)


                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(Secondstack.end())

                .strafeLeft(PARK)
                .back(7)

                .build();

        waitForStart();
        // Save computing resources by closing the main stream, since it's no longer needed.
        Cam1Portal.close();

        USE_TAGS = true;
        initAprilTag();
        telemetryAprilTag();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            switch (autoState) {
                case SPIKEMARK:

                    switch (side) {
                        case RIGHT:
                            if (!drive.isBusy()) {
                                drive.followTrajectorySequence(spikemarkRight);
                                autoState = AutoState.ONE_STACK;
                            }
                            break;
                        case LEFT:
                            if (!drive.isBusy()) {
                                drive.followTrajectorySequence(spikemarkLeft);
                                autoState = AutoState.ONE_STACK;
                            }
                            break;
                        case CENTER:
                            if (!drive.isBusy()) {
                                drive.followTrajectorySequence(spikemarkCenter);
                                autoState = AutoState.ONE_STACK;
                            }
                            break;
                    }
                case ONE_STACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(One_stack);
                        autoState = AutoState.BACKDROP_SCORE;
                    }
                    break;
                case BACKDROP_SCORE:
                    if (!drive.isBusy()) {
                        Pose2d currentPose = drive.getPoseEstimate();
                        drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
                        drive.followTrajectorySequence(Backdrop_score);
                    }
                    runThread(BackdropScoreThread);
                    break;
                case SECOND_STACK:

                case BACKDROP_SECOND_SCORE:

                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(park);
                    }
                    autoState = AutoState.IDLE;
                    break;

                case IDLE:
                    if (!drive.isBusy()) {
                        stop();
                    }
                    break;
            }

        }
    }

    // Team Prop Cam (front c920)
    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("Team_Prop.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("nothing", "Prop"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        Cam1Portal = myVisionPortalBuilder.build();
    }
    private int scanLocation() {
        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        if (JavaUtil.listLength(myTfodRecognitions) == 0) {
            Globals.LOCATION = 1;
            leds.LeftLightUp();
            side = Side.LEFT;
        } else {
            // Iterate through list and call a function to display info for each recognized object.
            for (Recognition myTfodRecognition_item2 : myTfodRecognitions) {
                myTfodRecognition = myTfodRecognition_item2;
                // Display info about the recognition.
                telemetry.addLine("");
                // Display label and confidence.
                // Display the label and confidence for the recognition.
                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
                // Display position.
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                // Display the position of the center of the detection boundary for the recognition
                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
                // Display size
                // Display the size of detection boundary for the recognition
                telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
                if (x < 90 && x > 35) {
                    Globals.LOCATION = 1;
                    side = Side.LEFT;
                    leds.LeftLightUp();
                    PARK = 30;
                } else if (x > 275 && x < 370) {
                    Globals.LOCATION = 2;
                    side = Side.CENTER;
                    PARK = 25;
                    leds.CenterLightUp();
                } else if (x > 500 && x < 620) {
                    Globals.LOCATION = 3;
                    side = Side.RIGHT;
                    leds.RightLightUp();
                    PARK = 20;
                }
            }
        }
        return Globals.LOCATION;
    }

    private void initAprilTag() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        if (USE_TAGS) {
            AprilTagPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam AprilTag"), aprilTagProcessor);
        } else {
            AprilTagPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, aprilTagProcessor);
        }
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection aprilTagDetection;

        // Grab list of AprilTag Detections
        myAprilTagDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));

        // Iterate through the list
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            aprilTagDetection = myAprilTagDetection_item;

            // Display info for Tags
            telemetry.addLine("");
            if (aprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + aprilTagDetection.id +") " + aprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(aprilTagDetection.ftcPose.x,6,1)
                        + " " + JavaUtil.formatNumber(aprilTagDetection.ftcPose.y,6,1) + " " +
                        JavaUtil.formatNumber(aprilTagDetection.ftcPose.z,6,1) + "  (inch)");

                telemetry.addLine("PRY " + JavaUtil.formatNumber(aprilTagDetection.ftcPose.pitch,6,1)
                        + " " + JavaUtil.formatNumber(aprilTagDetection.ftcPose.roll,6,1) + " " +
                        JavaUtil.formatNumber(aprilTagDetection.ftcPose.yaw,6,1) + "  (deg)");

                telemetry.addLine("RBE " + JavaUtil.formatNumber(aprilTagDetection.ftcPose.range,6,1)
                        + " " + JavaUtil.formatNumber(aprilTagDetection.ftcPose.bearing,6,1) + " " +
                        JavaUtil.formatNumber(aprilTagDetection.ftcPose.elevation,6,1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + aprilTagDetection.id + " ) Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(aprilTagDetection.center.x,6,0)
                        + " " + JavaUtil.formatNumber(aprilTagDetection.center.y,6,0)+" (pixels)");
            }

            if (aprilTagDetection.id == 3 && Globals.LOCATION == 3) {
                leds.RightLightUp();
            }   else if (aprilTagDetection.id == 2 && Globals.LOCATION == 2) {
                leds.CenterLightUp();
            }   else {
                leds.LeftLightUp();
            }
        }

        telemetry.addLine("");
        telemetry.addLine("Key: ");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    public Side getSide() {
        return side;
    }
}
