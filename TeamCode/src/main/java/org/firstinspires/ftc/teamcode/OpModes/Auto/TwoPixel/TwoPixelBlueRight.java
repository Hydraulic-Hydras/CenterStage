package org.firstinspires.ftc.teamcode.OpModes.Auto.TwoPixel;

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
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.common.Hardware.LEDS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "2 Pixel Blue Right", group = "2 Pixel")
public class TwoPixelBlueRight extends LinearOpMode {

    // FINALIZED N DONE
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
    public VisionPortal myVisionPortal;
    public double propLocation;
    public boolean USE_WEBCAM;
    public double x;
    public float y;

    // ** Useful **
    public Side side = Side.LEFT;

    @Override
    public void runOpMode() {
        Globals.IS_AUTO = true;

        // Put initialization blocks here.
        drive = new SampleMecanumDrive(hardwareMap);
        leds.initialize(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        intake.initialize(hardwareMap);

        USE_WEBCAM = true;
        initTfod();
        leds.loop();

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

        Pose2d startPose = Globals.StartPose;
        drive.setPoseEstimate(startPose);

        // TUNED AND FINISHED
        TrajectorySequence preloadCenter = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .addTemporalMarker(Intake::reverseIntake)
                .back(4)
                .strafeRight(16.5)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(Intake::stopIntaking)

                /*
                .splineToConstantHeading(new Vector2d(58, -18), Math.toRadians(-90))
                .lineTo(new Vector2d(58, 65))
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1300, 0.85))
                .splineToConstantHeading(new Vector2d(18.5, 70), Math.toRadians(-90))

                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .forward(3)
                .strafeLeft(29)
                .addTemporalMarker(() -> mitsumi.autoMoveTo(0, 0.55))
                .back(10)
                 */

                .build();

        // TUNED AND FINISHED
        TrajectorySequence preloadLeft = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .turn(Math.toRadians(90))
                .forward(4)
                .addTemporalMarker(Intake::reverseIntake)
                .back(10)
                .addTemporalMarker(Intake::stopIntaking)

                /*.strafeRight(30)
                .splineToConstantHeading(new Vector2d(61, 15), Math.toRadians(90))
                .forward(52)
                .turn(Math.toRadians(180))

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(20, 75), Math.toRadians(-90))
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 0.85))
                .back(4)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1.7)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> mitsumi.autoMoveTo(-200, 0.65))


                // park
                .waitSeconds(0.5)
                .strafeLeft(35)
                .back(5)

                 */

                .build();

        // TUNED AND FINISHED
        TrajectorySequence preloadRight = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                // Purple Pixel
                /*.forward(4)
                .addTemporalMarker(Intake::reverseIntake)
                .back(4)
                .addTemporalMarker(Intake::stopIntaking)
                 */

                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(-90)))
                .forward(4)
                .addTemporalMarker(Intake::reverseIntake)
                .back(4)
                .addTemporalMarker(Intake::stopIntaking)
/*
                .waitSeconds(0.5)
                .strafeLeft(29)
                .waitSeconds(5)
                .back(62)
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 0.85))


 */
                /*
                .strafeLeft(29)
                .back(65)

                // Scoring
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 0.85))
                .splineToConstantHeading(new Vector2d(23.5, 65), Math.toRadians(-90))
                .back(4)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(Intake::stopIntaking)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1.7)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> mitsumi.autoMoveTo(-200, 0.65))

                // Park
                .strafeLeft(24)
                .back(7)


                 */
                .build();

        waitForStart();
        // Put run blocks here.
        // Save computing resources by closing the camera stream, if no longer needed.
        myVisionPortal.close();

        if (isStopRequested()) return;
        // Start is pressed

        if (propLocation == 3) {
            drive.followTrajectorySequence(preloadRight);
        }   else if (propLocation == 2) {
            drive.followTrajectorySequence(preloadCenter);
        }   else {
            drive.followTrajectorySequence(preloadLeft);
        }
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
            Globals.LOCATION = 1;
            side = Side.LEFT;
            leds.LeftLightUp();
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
                } else if (x > 275 && x < 370) {
                    Globals.LOCATION = 2;
                    side = Side.CENTER;
                    leds.CenterLightUp();
                } else if (x > 500 && x < 620) {
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


