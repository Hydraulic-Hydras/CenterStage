package org.firstinspires.ftc.teamcode.Auto.Qual4;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
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
import org.firstinspires.ftc.teamcode.common.Hardware.LEDS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "RedLeft", group = "Qual 4")
public class RedLeft extends LinearOpMode {

    // Hardware
    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    private final LEDS leds = new LEDS(this);
    private SampleMecanumDrive drive;

    // Timer
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

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

        // LEFT (finished) 1/25/2024
        TrajectorySequence preloadLeft = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(30)
                .waitSeconds(0.1)
                .turn(Math.toRadians(90))
                .waitSeconds(0.1)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::reverseIntake)
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::stopIntaking)
                .back(2)

                // scoring
                .strafeRight(25)
                .splineToConstantHeading(new Vector2d(42, -50), Math.toRadians(90))
                .waitSeconds(0.01)
                .back(29)
                .waitSeconds(0.1)
                .lineTo(new Vector2d(18, -76.2))
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 1))
                .waitSeconds(0.9)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2.1)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> mitsumi.autoMoveTo(-150, 0.55))

                // park
                .strafeRight(30)
                .back(10)

                .build();

        // CENTER
        TrajectorySequence preloadCenter = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .strafeLeft(12)
                .forward(35.5)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::reverseIntake)
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::stopIntaking)
                .back(5)
                .strafeLeft(17)
                .waitSeconds(0.1)

                /*
                .strafeLeft(11)
                .turn(Math.toRadians(-180))
                .waitSeconds(0.5)
                //.lineTo(Globals.straightLineToPos)

                // Scoring
                //.splineToConstantHeading(Globals.curveSplineToBackDrop,  Math.toRadians(-180))
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 1))
                .waitSeconds(0.9)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2.1)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> mitsumi.autoMoveTo(-150, 0.55))

                // park
                .strafeRight(30)
\
                 */
                .build();

        // RIGHT (finished) 1/25/2024
        TrajectorySequence preloadRight = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .forward(29)
                .waitSeconds(0.1)
                .strafeLeft(2)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)
                .forward(5)
                .waitSeconds(0.1)
                .addTemporalMarker(Intake::reverseIntake)
                .waitSeconds(0.9)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, Intake::stopIntaking)

                .strafeLeft(30)
                .resetConstraints()

                .setConstraints(Globals.HalfVel, Globals.HalfAccel)
                .forward(73)
                .waitSeconds(0.1)
                .lineTo(new Vector2d(17, -75))
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .back(2)
                .waitSeconds(0.5)

                // Scoring
                .addTemporalMarker(() -> mitsumi.autoMoveTo(1250, 1))
                .waitSeconds(0.9)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> Intake.rotateBucket.setPosition(Intake.POS_DUMP))
                .waitSeconds(2.1)
                .addTemporalMarker(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> mitsumi.autoMoveTo(-150, 0.55))

                // park
                .strafeRight(30)

                .build();

        waitForStart();
        // Put run blocks here.
        // Save computing resources by closing the camera stream, if no longer needed.
        myVisionPortal.close();
        telemetry.addData("Side: ", getSide());
        telemetry.addData("Location: ", propLocation);

        if (isStopRequested()) return;
        // Start is pressed

        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        telemetry.addLine();

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
