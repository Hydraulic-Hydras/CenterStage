package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.CommandBase.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.common.CommandBase.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.common.CommandBase.ScoringCommand;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.LEDS;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "auto command Test")
public class CommandTest extends CommandOpMode {

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

    public double spikeX;
    public double spikeY;
    public double spikeH;
    public double backdropX;
    public double backdropY;
    public double backdropH;
    public double parkX;
    public double parkY;
    public double parkH;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
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

        drive.setPoseEstimate(Globals.RedRight_StartPoseBK);

        TrajectorySequence spikeMark = drive.trajectorySequenceBuilder(Globals.RedRight_StartPoseBK)
                .setConstraints(Globals.MaxVel, Globals.HalfAccel)

                //.lineToLinearHeading(new Pose2d(spikeX, spikeY, spikeH))
                //.splineToLinearHeading(new Pose2d(spikeX, spikeY), spikeH)

                .lineTo(new Vector2d(spikeX, spikeY))
                .turn(spikeH)

                .build();

        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(spikeMark.end())
                .setConstraints(Globals.MaxVel, Globals.HalfAccel)

                .back(7)
                .lineToLinearHeading(new Pose2d(backdropX, backdropY, backdropH))

                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(backdrop.end())
                .setConstraints(Globals.MaxVel, Globals.HalfAccel)

                .lineToLinearHeading(new Pose2d(parkX, parkY, parkH))

                .build();

        switch (side) {
            case LEFT:
                spikeX = 12.5;
                spikeY = -30;
                spikeH = Math.toRadians(90);

                backdropX = 46;
                backdropY = -27.5;
                backdropH = Math.toRadians(180);

            case CENTER:
                spikeX = 12.5;
                spikeY = -32;
                spikeH = Math.toRadians(0);

                backdropX = 46;
                backdropY = -33;
                backdropH = Math.toRadians(180);

            case RIGHT:
                spikeX = 34;
                spikeY = -37;
                spikeH = Math.toRadians(0);

                backdropX = 46;
                backdropY = -43;
                backdropH = Math.toRadians(180);
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new InstantCommand(() -> drive.followTrajectorySequence(spikeMark)),
                        new WaitCommand(500),
                        new IntakeReverseCommand(),
                        new WaitCommand(500),

                        new IntakeStopCommand(),
                        new WaitCommand(100),

                        new InstantCommand(() -> drive.followTrajectorySequence(backdrop))
                                .alongWith(new ScoringCommand()),

                        new WaitCommand(500)

                      //  new InstantCommand(() -> drive.followTrajectorySequenceAsync(park))
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
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
