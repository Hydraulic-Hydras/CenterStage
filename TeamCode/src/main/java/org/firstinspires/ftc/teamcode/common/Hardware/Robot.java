package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Climber;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.LEDS;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class Robot extends Contraption {

    public Mitsumi mitsumi = new Mitsumi(opMode);
    public Intake intake = new Intake(opMode);
    public LEDS leds = new LEDS(opMode);
    public Climber climber = new Climber(opMode);
    public SampleMecanumDrive drive;

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

    public Robot(LinearOpMode opMode) { this.opMode = opMode; }
    @Override
    public void initialize(HardwareMap hwMap) {
        Globals.IS_AUTO = true;

        drive = new SampleMecanumDrive(hwMap);
        mitsumi.initialize(hwMap);
        mitsumi.autoInit();
        intake.initialize(hwMap);
        leds.initialize(hwMap);
        climber.initialize(hwMap);

        USE_WEBCAM = true;
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("Red Prop (States).tflite");
        myTfodProcessorBuilder.setModelFileName("Blue Prop (States).tflite");
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
            myVisionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        Cam1Portal = myVisionPortalBuilder.build();

        propLocation = scanLocation();

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

    public void setPoseEstimate(Pose2d pose2d) {
        drive.setPoseEstimate(pose2d);
    }

    public void getPoseEstimate() {
        drive.getPoseEstimate();
    }

    public double returnLocation() {
        return propLocation;
    }

    public Side getSide() {
        return side;
    }

    public void stopStreaming() {
        Cam1Portal.stopStreaming();
    }
}
