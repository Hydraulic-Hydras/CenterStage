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
public class RedRight extends LinearOpMode {
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

        // RED RIGHT
        drive.setPoseEstimate(Globals.RedRight_StartPoseBK);

        TrajectorySequence Right = drive.trajectorySequenceBuilder(Globals.RedRight_StartPoseBK)
                .setConstraints(Globals.MaxVel, Globals.HalfAccel)

                .addTemporalMarker(Intake::retainerClose)

                // preload
                .splineToConstantHeading(new Vector2d(28, -42), Math.toRadians(90))
                .addTemporalMarker(Intake::reverseIntake)
                .waitSeconds(0.5)

                .back(8)
                .addTemporalMarker(Intake::stopIntaking)
                .turn(Math.toRadians(92))
                .resetConstraints()

                // backdrop
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1100, 0.65))
                .lineTo(new Vector2d(53.5, -43))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(Intake::retainerOpen)
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1.5)
                .forward(4)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))

                // park
                .lineTo(new Vector2d(48, -65))
                .lineTo(new Vector2d(55, -65))

                .build();

        TrajectorySequence Center = drive.trajectorySequenceBuilder(Globals.RedRight_StartPoseBK)
                .setConstraints(Globals.MaxVel, Globals.HalfAccel)

                // preload
                .lineToLinearHeading(new Pose2d(12.5, -34.5, Math.toRadians(90)))
                .addTemporalMarker(Intake::reverseIntake)
                .waitSeconds(0.1)

                .back(7)
                .addTemporalMarker(Intake::stopIntaking)
                .turn(Math.toRadians(90))
                .waitSeconds(1)

                // backdrop
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1100, 0.65))
                .resetConstraints()
                .lineTo(new Vector2d(46, -33))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1.5)
                .forward(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))

                // park
                .lineTo(new Vector2d(42, -64))
                .lineTo(new Vector2d(50, -64))

                .build();

        TrajectorySequence Left = drive.trajectorySequenceBuilder(Globals.RedRight_StartPoseBK)
                .setConstraints(Globals.MaxVel, Globals.HalfAccel)

                // preload
                .forward(32)
                .turn(Math.toRadians(90))
                .addTemporalMarker(Intake::reverseIntake)
                .forward(3)
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(Intake::stopIntaking)

                // backdrop
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1100, 0.65))
                .resetConstraints()
                .lineTo(new Vector2d(46, -27.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1.6)
                .forward(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))

                // park
                .lineTo(new Vector2d(42, -64))
                .lineTo(new Vector2d(50, -64))

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

                if (x < 50 && x > 25) {
                    Globals.LOCATION = 1;
                    side = Side.LEFT;
                    leds.LeftLightUp();
                } else if (x > 260 && x < 300) {
                    Globals.LOCATION = 2;
                    side = Side.CENTER;
                    leds.CenterLightUp();
                } else if (x > 520 && x < 580) {
                    Globals.LOCATION = 3;
                    side = Side.RIGHT;
                    leds.RightLightUp();
                }

            }
        }

        return Globals.LOCATION;
    }

    public Side getSide() {
        return side;
    }
}
