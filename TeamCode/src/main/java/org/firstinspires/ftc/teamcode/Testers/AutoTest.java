package org.firstinspires.ftc.teamcode.Testers;

public class AutoTest{} /* extends CommandOpMode {

    public Intake intake = new Intake(this);
    public Mitsumi mitsumi = new Mitsumi(this);

    public SampleMecanumDrive drive;

    public List<Recognition> myTfodRecognitions;
    public TfodProcessor myTfodProcessor;
    public Recognition myTfodRecognition;
    public VisionPortal myVisionPortal;
    public int location;
    public boolean USE_WEBCAM;
    public double x;
    public float y;

    public Side side = Side.LEFT;

    // Timer
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    // Trajectories
    public static TrajectorySequence yellowPixelDeposit;
    public static TrajectorySequence purplePixelDeposit;
    public static TrajectorySequence PurplePixelDeposit1;
    public static TrajectorySequence park;

    public static double tX;
    public static double tY;
    public static double tH;

    public static double sX;
    public static double sY;
    public static double sH;

    public double[] distancePark = {17, 25, 27};
    public double distance = 0;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        double propLocation;

        drive = new SampleMecanumDrive(hardwareMap);
        intake.initialize(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        USE_WEBCAM = true;
        initTfod();

        // Put initialization blocks here.
        telemetry.addLine("Robot initialization in process...");
        telemetry.addLine("Please  DO NOT press anything!!!");
        telemetry.update();

        while (!isStarted()) {
            propLocation = scanLocation();
            telemetry.addLine("");
            telemetry.addData("Team Prop Location", propLocation);
            telemetry.addData("Side: ", getSide());
            telemetry.addLine("");
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addLine("Press DpadDown to stop streaming");
            telemetry.addLine("Press DpadUp to resume streaming");
            telemetry.update();

            if (gamepad1.dpad_down) {
                // Temporarily stop the streaming session.
                myVisionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                // Resume the streaming session if previously stopped.
                myVisionPortal.resumeStreaming();
            }
            // Share the CPU.
            sleep(20);
        }

        // Put run blocks here.
        // Save computing resources by closing the camera stream, if no longer needed.
        myVisionPortal.close();

        Pose2d startPose = (Globals.StartPoseRED_RIGHT);

        purplePixelDeposit = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .lineToLinearHeading(new Pose2d(tX, tY, tH))

                .build();

        // only for right prop
        PurplePixelDeposit1 = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .splineToConstantHeading(new Vector2d(23, -40), Math.toRadians(90))

                .build();

        yellowPixelDeposit = drive.trajectorySequenceBuilder(purplePixelDeposit.end())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .lineToLinearHeading(new Pose2d(sX, sY, sH))

                .build();

        // Park
        park = drive.trajectorySequenceBuilder(new Pose2d())
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)
                .strafeLeft(distance)
                .back(8)

                .build();

        switch (side) {
            case CENTER:
                tX = Globals.RED_SPIKEMARK_RIGHT_X;
                tY = Globals.RED_SPIKEMARK_RIGHT_Y;
                tH = Globals.NORTH;

                sX = 52;
                sY = -33.5;
                sH = Globals.WEST;

                distance = distancePark[2];
                break;
            case LEFT:
                tX = Globals.RED_SPIKEMARK_RIGHT_X;
                tY = Globals.RED_SPIKEMARK_RIGHT_Y;
                tH = Globals.WEST;

                sX = 52;
                sY = -30;
                sH = Globals.WEST;

                distance = distancePark[3];
                break;
            case RIGHT:
                // test to see if this method works and overloads the old trajectory
                purplePixelDeposit = PurplePixelDeposit1;

                sX = 52;
                sY = -37.5;
                sH = Globals.WEST;

                distance = distancePark[1];
                break;
        }

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(timer::reset),

                                new InstantCommand(() -> drive.followTrajectorySequence(purplePixelDeposit))
                                        .alongWith(new IntakeReverseCommand()),

                                new WaitCommand(700),
                                new IntakeStopCommand(),

                                new InstantCommand(() -> drive.followTrajectorySequence(yellowPixelDeposit))
                                        .alongWith(new LiftCommand(1500, 1)),

                                new WaitCommand(900),
                                new OuttakeCommand(),

                                new LiftCommand(100, 0.65)
                                        .alongWith(new InstantCommand(() -> drive.followTrajectorySequence(park))),

                                new InstantCommand(() -> endTime = timer.seconds())
                        )
                );
    }

    @Override
    public void run() {
        super.run();

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        telemetry.addLine();

        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
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

    /**
     * Display info (using telemetry) for a detected object
     */
/**

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

**/