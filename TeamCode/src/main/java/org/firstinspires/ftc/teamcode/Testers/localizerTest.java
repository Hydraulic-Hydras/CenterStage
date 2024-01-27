package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (group = "Testers")
public class localizerTest extends LinearOpMode {

    private SampleMecanumDrive drive;

    /**
     * This Class is being used to eliminate the 12.72
     *  -> New calculation theorizes that there is a 15.02 second wait time during the whole auto
     *  cut down to only 5.02 seconds and a 2 + 4 can be achieved
     */
    @Override
    public void runOpMode() {
        Globals.IS_AUTO = true;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12.5, -62, Math.toRadians(90)));

        while (!isStopRequested() || opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        // add movement here
        TrajectorySequence localize = drive.trajectorySequenceBuilder(new Pose2d(12.5, -62, Math.toRadians(90)))
                .setConstraints(Globals.MaxVel, Globals.MaxAccel)



                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(localize);
    }
}
