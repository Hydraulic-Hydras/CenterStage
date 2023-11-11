package org.firstinspires.ftc.teamcode.CenterStage.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CenterStage.RedConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "LEFT RED", group = "RED")
public class Left_Auto extends LinearOpMode {

    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(RedConstants.LEFT_SP);

        TrajectorySequence RIGHT_Prop = drive.trajectorySequenceBuilder(RedConstants.LEFT_SP)

                .setConstraints(RedConstants.Vel0, RedConstants.Accel0)
                .setTurnConstraint(60, 60)

                .lineToLinearHeading(RedConstants.L_LEFT_PROP_UNLOAD_POSE)
                .waitSeconds(1)

                .strafeRight(1)
                .splineToConstantHeading(RedConstants.UPPER_WHITE_STACK_VECTOR, RedConstants.HEADING_HIGH_PIXEL)

                .waitSeconds(5)

                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(RIGHT_Prop);
        }
    }
}
