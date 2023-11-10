package org.firstinspires.ftc.teamcode.CenterStage.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CenterStage.AutoConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auto extends LinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPoseEstimate(AutoConstants.LEFT_SP);

        TrajectorySequence RIGHT_Prop = drive.trajectorySequenceBuilder(AutoConstants.LEFT_SP)

                .setConstraints(AutoConstants.Vel0, AutoConstants.Accel0)
                .setTurnConstraint(60, 60)

                .forward(5)

                .strafeLeft(10.7)

                /**
                .lineTo(new Vector2d(-46.7, 35.3))

                .setTurnConstraint(50, 50)
                .turn(Math.PI / -2)
                .resetTurnConstraint()

                .lineTo(new Vector2d(-58.3, 35.3))
                .waitSeconds(1)

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-40, 12, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(24, 12))
                .splineToConstantHeading(new Vector2d(50, 41), Math.toRadians(0))

                 **/

                .waitSeconds(5)


                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(RIGHT_Prop);
        }
    }
}
