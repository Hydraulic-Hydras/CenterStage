package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "RedRightBK", group = "Red")
public class RedRightBk extends LinearOpMode {

    // Hardware
    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    private SampleMecanumDrive drive;

    // Timer
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    // Vision
    public List<Recognition> myTfodRecognitions;
    public TfodProcessor myTfodProcessor;
    public Recognition myTfodRecognition;
    public VisionPortal myVisionPortal;
    public int location;
    public double propLocation;
    public boolean USE_WEBCAM;
    public double x;
    public float y;

    // ** Useful **
    public Side side = Side.LEFT;

    @Override
    public void runOpMode() {
        // Put initialization blocks here.
        drive = new SampleMecanumDrive(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        intake.initialize(hardwareMap);

        USE_WEBCAM = true;
        initTfod();

        // Telemetry warning
        telemetry.addLine("Robot initialization in process...");
        telemetry.addLine("Do not press or move anything as Robot will move!!!");
        telemetry.update();

        while (!isStarted()) {
            propLocation = scanLocation();
            telemetry.addData("Team Prop Location", propLocation);
            telemetry.addData("Side: ", getSide());
            telemetry.update();
        }

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // CENTER
        TrajectorySequence preloadCenter = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(29)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::reverseIntake)
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::stopIntaking)
                .waitSeconds(0.1)
                .back(5)
                .turn(Math.toRadians(90))
                .back(30.5)

                // Scoring
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> mitsumi.autoMoveTo(1250, 1))
                .waitSeconds(0.9)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> mitsumi.autoMoveTo(0, 0.55))

                // park
                .strafeLeft(17)
                .back(10)
                .build();

        // RIGHT
        TrajectorySequence preloadRight = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .splineToConstantHeading(new Vector2d(19.5, -8.6), Math.toRadians(0))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::reverseIntake)
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::stopIntaking)
                .waitSeconds(0.1)
                .back(2.5)

                .resetConstraints()

                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(17, -27.8))

                // Scoring
                .waitSeconds(0.5)
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 1))
                .waitSeconds(0.9)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> mitsumi.autoMoveTo(0, 0.55))

                // park
                .strafeLeft(10)
                .back(10)

                .build();

        // LEFT (finished)
        TrajectorySequence preloadLeft = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(29)
                .turn(Math.toRadians(90))
                .forward(3)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::reverseIntake)
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::stopIntaking)
                .waitSeconds(0.1)
                .back(33.5)
                .strafeRight(3.5)

                // Scoring
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> mitsumi.autoMoveTo(1250, 1))
                .waitSeconds(0.9)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> mitsumi.autoMoveTo(0, 0.55))

                // park
                .strafeLeft(22)
                .back(8)

                .build();

        waitForStart();
        // Put run blocks here.
        // Save computing resources by closing the camera stream, if no longer needed.
        myVisionPortal.close();
        telemetry.addData("Side: ", getSide());
        telemetry.addData("Location: ", propLocation);

        if (isStopRequested()) return;
        // Start is pressed

        if (propLocation == 3) {
            telemetry.addLine("Running Path for Right Prop");
            drive.followTrajectorySequence(preloadRight);
        }   else if (propLocation == 2) {
            telemetry.addLine("Running Path for Center Prop");
            drive.followTrajectorySequence(preloadCenter);
        }   else {
            telemetry.addLine("No Prop detected, Running default Path for Left Prop");
            drive.followTrajectorySequence(preloadLeft);
        }

        telemetry.addLine();
        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        telemetry.update();
    }

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
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private int scanLocation() {
        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        if (JavaUtil.listLength(myTfodRecognitions) == 0) {
            location = 1;
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
                if (x > 460 && x < 600) {
                    // Location 1 : 460 < x < 600
                    location = 3;
                    side = Side.RIGHT;
                } else if (x > 45 && x < 330) {
                    // Location 2 : 45 < x <330
                    location = 2;
                    side = Side.CENTER;
                } else {
                    location = 1;
                    side = Side.LEFT;
                }
            }
        }
        return location;

    }
    public Side getSide() {
        return side;
    }

}
