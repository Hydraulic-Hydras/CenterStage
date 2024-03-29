package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.LEDS;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled
@Autonomous
public class AprilTagTest extends LinearOpMode {

    // Vision
    public List<Recognition> myTfodRecognitions;
    public TfodProcessor myTfodProcessor;
    public Recognition myTfodRecognition;
    public VisionPortal myVisionPortal;
    public double propLocation;
    public boolean USE_WEBCAM;
    public double x;
    public float y;
    // AprilTags
    public boolean USE_TAGS;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal AprilTagPortal;
    public int tagLocation;
    public static int ID;
    // ** Useful **
    public Side side = Side.LEFT;
    // Led
    private final LEDS leds = new LEDS(this);
    @Override
    public void runOpMode() {
        Globals.IS_AUTO = true;

        USE_TAGS = true;
        leds.initialize(hardwareMap);

        initTfod();

        while (!isStarted()) {
            propLocation = scanLocation();
            telemetry.addData("Team Prop Location", propLocation);
            telemetry.addData("Side: ", getSide());
            telemetry.update();
        }

        waitForStart();
        myVisionPortal.close();
        initAprilTag();

        while (opModeIsActive()) {
            ID = telemetryAprilTag();
            telemetry.update();
        }
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

            tagLocation = aprilTagDetection.id;

            if (Globals.LOCATION == 3) {
                side = Side.RIGHT;
                leds.RightLightUp();
                tagLocation = 6;
            }   else if (Globals.LOCATION == 2) {
                side = Side.CENTER;
                leds.CenterLightUp();
                tagLocation = 5;
            }   else if (Globals.LOCATION == 1) {
                side = Side.LEFT;
                leds.LeftLightUp();
                tagLocation = 4;
            }   else {
                side = Side.LEFT;
                leds.LeftLightUp();
                tagLocation = 4;
            }

        }

        telemetry.addLine("");
        telemetry.addLine("Key: ");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return tagLocation;
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
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Prop Webcam"));
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
