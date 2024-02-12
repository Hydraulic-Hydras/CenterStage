package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.CenterStage.Tag;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.LEDS;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled
@Autonomous (name = "APR Red Right APR", group = " 2 + 4")
public class AprilTagTrajectoryTest extends LinearOpMode {

    // Hardware Setup
    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    public SampleMecanumDrive drive;

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
    public double TagLocation;
    public boolean tagFound = false;

    public Runnable ScoringBackdrop = () -> {
        try {
            mitsumi.autoMoveTo(1250, 1);
            Intake.rotateBucket.setPosition(Intake.POS_PANEL);
            Thread.sleep(500);
            Intake.rotateBucket.setPosition(Intake.POS_DUMP);
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable retract = () -> {
      try {
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

    // ** Useful **
    public Side side = Side.LEFT;
    public Tag tag = Tag.ONE;

    public enum Movement {
        SpikeMark,
        Backdrop,
        Park,
        Idle
    }

    public Movement movement = Movement.SpikeMark;

    public double X, Y, H;
    public double parkX;
    public double parkY;
    public double parkH;

    public ElapsedTime timer = new ElapsedTime();
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
        Thread Retraction = new Thread(retract);

        // RED RIGHT
        drive.setPoseEstimate(Globals.StartPose);

        TrajectorySequence Right = drive.trajectorySequenceBuilder(Globals.StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .splineToConstantHeading(new Vector2d(19.5, -15), Math.toRadians(0))
                .addTemporalMarker(Intake::reverseIntake)

                .lineTo(new Vector2d(15.5, -15))
                .addTemporalMarker(Intake::stopIntaking)

                // Temp Park
                .setReversed(true)
                .splineTo(new Vector2d(29, -25), Math.toRadians(90))

                .build();

        TrajectorySequence Center = drive.trajectorySequenceBuilder(Globals.StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .lineTo(new Vector2d(29, 0))
                .addTemporalMarker(Intake::reverseIntake)

                .lineTo(new Vector2d(25, 0))
                .addTemporalMarker(Intake::stopIntaking)

                // Temp Park
                .setReversed(true)
                .splineTo(new Vector2d(29, -25), Math.toRadians(90))

                .build();

        TrajectorySequence Left = drive.trajectorySequenceBuilder(Globals.StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(90)))
                .addTemporalMarker(Intake::reverseIntake)

                .lineTo(new Vector2d(36, -10))
                .addTemporalMarker(Intake::stopIntaking)

                // Temp Park
                .setReversed(true)
                .splineTo(new Vector2d(29, -25), Math.toRadians(90))

                .build();

        TrajectorySequence backDropDisplacement = drive.trajectorySequenceBuilder(new Pose2d())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .setTangent(0)
                .splineToConstantHeading(new Vector2d(X, Y), H)

                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d())

                // Based on Prop Location values change
                .lineToLinearHeading(new Pose2d(parkX, parkY, parkH))

                .build();
        waitForStart();
        // Save computing resources by closing the main stream, since it's no longer needed.
        Cam1Portal.close();

        USE_TAGS = true;

        initAprilTag();
        TagLocation = telemetryAprilTag();

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            telemetry.addData("Tag: ", getTag());
            telemetry.update();

            switch (movement) {
                case SpikeMark:
                    if (propLocation == 3) {
                        drive.followTrajectorySequence(Right);
                        movement = Movement.Backdrop;
                    } else if (propLocation == 2) {
                        drive.followTrajectorySequence(Center);
                        movement = Movement.Backdrop;
                    } else {
                        drive.followTrajectorySequence(Left);
                        movement = Movement.Backdrop;
                    }
                    break;

                case Backdrop:
                    if (!drive.isBusy()) {
                        if (getSide() == Side.RIGHT && TagLocation == 3) {
                            tag = Tag.THREE;
                            drive.followTrajectorySequence(backDropDisplacement);
                            runThread(BackdropScoreThread);
                            movement = Movement.Park;
                        } else if (getSide() == Side.CENTER && TagLocation == 2) {
                            tag = Tag.TWO;
                            drive.followTrajectorySequence(backDropDisplacement);
                            runThread(BackdropScoreThread);
                            movement = Movement.Park;
                        } else {
                            tag = Tag.ONE;
                            drive.followTrajectorySequence(backDropDisplacement);
                            runThread(BackdropScoreThread);
                            movement = Movement.Park;
                        }

                    }
                    break;

                case Park:
                    if (!drive.isBusy()) {
                        runThread(Retraction);
                        drive.followTrajectorySequence(park);
                        movement = Movement.Idle;
                    }
                    break;

                case Idle:
                    if (!drive.isBusy() && timer.seconds() > 1.5) {
                        stop();
                    }
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

            parkX = -1;
            parkY = 30;
            parkH = Math.toRadians(0);
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

                    parkX = -1;
                    parkY = 30;
                    parkH = Math.toRadians(0);

                } else if (x > 275 && x < 370) {
                    Globals.LOCATION = 2;
                    side = Side.CENTER;
                    leds.CenterLightUp();

                    parkX = -1;
                    parkY = 25;
                    parkH = Math.toRadians(0);

                } else if (x > 500 && x < 620) {
                    Globals.LOCATION = 3;
                    side = Side.RIGHT;
                    leds.RightLightUp();

                    parkX = -1;
                    parkY = 15;
                    parkH = Math.toRadians(0);
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

    private int telemetryAprilTag() {
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
                tagFound = true;
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
                tagFound = false;
                telemetry.addLine("==== (Tag ID not Found");
                telemetry.addLine("==== (ID " + aprilTagDetection.id + " ) Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(aprilTagDetection.center.x,6,0)
                        + " " + JavaUtil.formatNumber(aprilTagDetection.center.y,6,0)+" (pixels)");
            }

            Globals.TAG = aprilTagDetection.id;
        }

        if (Globals.TAG == 3 && Globals.LOCATION == 3) {
            X = 0;
            Y = 0;
            H = Math.toRadians(0);
        }   else if (Globals.TAG == 2 && Globals.LOCATION == 2) {
            X = 0;
            Y = 0;
            H = Math.toRadians(0);
        }   else {
            X = 0;
            Y = 0;
            H = Math.toRadians(0);
        }

        telemetry.addLine("");
        telemetry.addLine("Key: ");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return Globals.TAG;
    }

    public Side getSide() {
        return side;
    }

    public Tag getTag() {
        return tag;
    }
}
