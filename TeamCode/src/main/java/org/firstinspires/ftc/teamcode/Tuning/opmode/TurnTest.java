package org.firstinspires.ftc.teamcode.Tuning.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}
