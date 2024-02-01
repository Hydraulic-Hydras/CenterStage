package org.firstinspires.ftc.teamcode.OpModes.Auto.FourPixel;

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
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled
@Autonomous (name = "2 + 2 BlueRight", group = "2 + 2")
public class BlueRight extends LinearOpMode {

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

        TrajectorySequence preloadCenter = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .addTemporalMarker(Intake::reverseIntake)
                .back(4)
                .strafeRight(16.5)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(Intake::stopIntaking)
                .splineToConstantHeading(new Vector2d(58, -18), Math.toRadians(-90))

                // Theoretically this should work
                .forward(4)
                .addTemporalMarker(Intake::fingerDown)
                .waitSeconds(1)
                .back(8)
                .addTemporalMarker(Intake::fingerReset)
                .forward(9.5)
                .addTemporalMarker(Intake::startIntaking)
                .waitSeconds(0.5)

                // Lets see if i can make this into a 2 + 1 as well for funsies yk
                .lineTo(new Vector2d(58, 65))
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1300, 0.85))
                .splineToConstantHeading(new Vector2d(18.5, 70), Math.toRadians(-90))

                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .forward(3)
                .strafeLeft(29)
                .addTemporalMarker(() -> mitsumi.autoMoveTo(0, 0.55))
                .back(10)

                .build();

        TrajectorySequence preloadLeft = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .forward(29)
                .turn(Math.toRadians(90))
                .forward(4)
                .addTemporalMarker(Intake::reverseIntake)
                .back(10)
                .addTemporalMarker(Intake::stopIntaking)

                .splineToConstantHeading(new Vector2d(58, -15), Math.toRadians(90))
                .lineTo(new Vector2d(58, 65))
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1300, 0.85))
                .splineToConstantHeading(new Vector2d(12.5, 70), Math.toRadians(90))


                .build();

        TrajectorySequence preloadRight = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                // Purple Pixel
                .forward(29)
                .turn(Math.toRadians(-90))
                .forward(4)
                .addTemporalMarker(Intake::reverseIntake)
                .back(4)
                .addTemporalMarker(Intake::stopIntaking)

                // Pick up from stack
                .strafeLeft(29)
                .forward(21)
                .addTemporalMarker(Intake::fingerDown)
                .waitSeconds(1)
                .back(8)
                .addTemporalMarker(Intake::fingerReset)
                .forward(9.5)
                .addTemporalMarker(Intake::startIntaking)
                .waitSeconds(0.5)
                .back(85)

                // Scoring
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1450, 0.65))
                .splineToConstantHeading(new Vector2d(26, 74.5), Math.toRadians(-90))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(Intake::stopIntaking)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(1)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> mitsumi.autoMoveTo(-200, 0.65))

                // Park
                .strafeLeft(15)
                .back(7)

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

                if (side == null) {
                    side = Side.LEFT;
                    leds.LeftLightUp();
                }

            }
        }
        return Globals.LOCATION;

    }
    public Side getSide() {
        return side;
    }

}
