package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous (group = "Testers")
public class localizerTest extends LinearOpMode {

    private SampleMecanumDrive drive;

    /**
     * This Class is being used to eliminate the 12.72
     * -> New calculation theorizes that there is a 15.02 second wait time during the whole auto
     * cut down to only 5.02 seconds and a 2 + 4 can be achieved
     */
    @Override
    public void runOpMode() {
        Globals.IS_AUTO = true;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12.5, -62, Math.toRadians(90)));

        Pose2d poseEstimate = drive.getPoseEstimate();

        while (!isStopRequested()) {
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        // add movement here
        TrajectorySequence localize = drive.trajectorySequenceBuilder(new Pose2d(12.5, -62, Math.toRadians(90)))
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)

                .lineTo(new Vector2d(12.5, -33)).back(5)
                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(43.7, -38))

                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(35.5, -7), Math.toRadians(180))
                .lineTo(new Vector2d(-31.3, -7))

                .splineToConstantHeading(new Vector2d(-55, -11.6), Math.toRadians(180))
                .forward(1.2)

                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                .waitSeconds(0.5)

                .splineToConstantHeading(new Vector2d(43.7, -35), Math.toRadians(180))

                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(35.5, -7), Math.toRadians(180))
                .lineTo(new Vector2d(-31.3, -7))

                .splineToConstantHeading(new Vector2d(-55, -11.6), Math.toRadians(180))
                .forward(1.2)

                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180)))
                .waitSeconds(0.5)

                .splineToConstantHeading(new Vector2d(43.7, -35), Math.toRadians(180))

                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.followTrajectorySequence(localize);
        }


    }
}
