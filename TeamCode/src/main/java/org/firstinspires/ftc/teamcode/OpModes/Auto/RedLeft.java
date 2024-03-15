package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.LEDS;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (group = "Red")
public class RedLeft extends LinearOpMode {

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

    /** useful **/
    public Side side = Side.LEFT;

    @Override
    public void runOpMode() {
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

        drive.setPoseEstimate(Globals.RedLeft_StartPose);

        // just leave it be
        TrajectorySequence Left = drive.trajectorySequenceBuilder(Globals.RedLeft_StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .addTemporalMarker(Intake::retainerClose)

                // spikemark
                .lineToLinearHeading(new Pose2d(-35.5, -34, Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .addTemporalMarker(Intake::reverseIntake)
                .forward(4)
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(Intake::stopIntaking)

                .setTangent(Math.PI)
                .strafeRight(10)
                .splineToConstantHeading(new Vector2d(-60, -19.5), Math.toRadians(180))
                .forward(3.5)
                .addTemporalMarker(Intake::fingerDown)
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(Intake::startIntaking)
                .forward(8.5)
                .waitSeconds(0.5)

                .lineTo(new Vector2d(-55, -12))
                .lineTo(new Vector2d(29.5, -12))
                .addTemporalMarker(Intake::stopIntaking)
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 1))
                .lineTo(new Vector2d(29.5, -37))
                .back(5.5) // CHANGE ONLY THIS IF ANYTHING
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::retainerOpen)
                .waitSeconds(1)
                .forward(2)
                .waitSeconds(0.5)
                .back(2)
                .waitSeconds(0.5)
                .addTemporalMarker(Intake::bucketRest)

                // park
                .lineTo(new Vector2d(36, -12))
                .lineTo(new Vector2d(50, -12))

                .build();

        // YOU ARE A PAIN IN THE ASS <3
        TrajectorySequence Center = drive.trajectorySequenceBuilder(Globals.RedLeft_StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .addTemporalMarker(Intake::retainerClose)

                // spikemark
                .lineToLinearHeading(new Pose2d(-35.5, -36.5, Math.toRadians(90)))
                .addTemporalMarker(Intake::reverseIntake)
                .waitSeconds(0.7)
                .back(7)
                .addTemporalMarker(Intake::stopIntaking)
                .strafeLeft(22)

                // stack
                .lineTo(new Vector2d(-55.5, -19.3))
                .waitSeconds(0.5)
                .turn(Math.toRadians(90))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-64.2, -19.3))
                .addTemporalMarker(Intake::fingerDown)
                .waitSeconds(0.5)
                .back(6.5)
                .addTemporalMarker(Intake::fingerReset)
                .addTemporalMarker(Intake::startIntaking)
                .forward(8)
                .waitSeconds(1)
                .lineTo(new Vector2d(-60, -14))

                // under the door
                .lineTo(new Vector2d(29.5, -14.3))
                .addTemporalMarker(Intake::stopIntaking)
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 1))
                .lineTo(new Vector2d(29.5, -46))
                .back(6) // CHANGE ONLY THIS IF ANYTHING
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::retainerOpen)
                .waitSeconds(1)
                .forward(2)
                .addTemporalMarker(Intake::bucketRest)

                // park
                .lineTo(new Vector2d(29.5, -19.3))
                .lineTo(new Vector2d(50, -12))

                .build();

        // Heavily despise this but WE BALL
        TrajectorySequence Right = drive.trajectorySequenceBuilder(Globals.RedLeft_StartPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .addTemporalMarker(Intake::retainerClose)

                // spikemark
                .lineTo(new Vector2d(-35.5, -34))
                .turn(Math.toRadians(-90))
                .addTemporalMarker(Intake::reverseIntake)
                .forward(4)
                .waitSeconds(0.5)
                .back(8)
                .addTemporalMarker(Intake::stopIntaking)

                // stack
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-45.5, -34))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-60.5, -34))
                .addTemporalMarker(Intake::fingerDown)
                .waitSeconds(0.5)
                .back(6)
                .addTemporalMarker(Intake::startIntaking)
                .forward(7.5)
                .waitSeconds(1)
                .back(10)

                // under the door
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(-55, -13), Math.toRadians(180))
                .lineTo(new Vector2d(29, -13))
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 1))
                .lineTo(new Vector2d(29, -45))
                .back(5.5) // CHANGE ONLY THIS IF ANYTHING
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::retainerOpen)
                .waitSeconds(1)
                .forward(2)
                .addTemporalMarker(Intake::bucketRest)

                // park
                .lineTo(new Vector2d(29.5, -19.3))
                .lineTo(new Vector2d(50, -14))

                .build();

        waitForStart();
        // Save computing resources by closing the main stream, since it's no longer needed.
        Cam1Portal.stopStreaming();

        if (propLocation == 3) {
            drive.followTrajectorySequence(Right);
        } else if (propLocation == 2) {
            drive.followTrajectorySequence(Center);
        } else {
            drive.followTrajectorySequence(Left);
        }
    }

    // Team Prop Cam (front c920)
    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("Red Prop (States).tflite");
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
            leds.noDetectLightUp();
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

                if (x < 80) {
                    Globals.LOCATION = 1;
                    side = Side.LEFT;
                    leds.LeftLightUp();
                } else if (x > 500) {
                    Globals.LOCATION = 3;
                    side = Side.RIGHT;
                    leds.RightLightUp();
                } else {
                    Globals.LOCATION = 2;
                    side = Side.CENTER;
                    leds.CenterLightUp();
                }

            }
        }

        return Globals.LOCATION;
    }

    public Side getSide() {
        return side;
    }
}
