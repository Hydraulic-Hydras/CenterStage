package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;

@Config
public class Globals {

    public static boolean IS_AUTO = false;
    public static int LOCATION;
    public static int TAG;

    // VELOCITY AND ACCELERATION
    public static final TrajectoryVelocityConstraint MaxVel = SampleMecanumDrive.getVelocityConstraint(
            100, Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint MaxAccel = SampleMecanumDrive.getAccelerationConstraint(
            100);

    public static final TrajectoryVelocityConstraint HalfVel = SampleMecanumDrive.getVelocityConstraint(
            90, Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint HalfAccel = SampleMecanumDrive.getAccelerationConstraint
            (65);

    // Start Position
    public static final Pose2d StartPose = new Pose2d(0, 0, Math.toRadians(0));

    // New StartPose
    public static final Pose2d RedRight_StartPoseBK = new Pose2d(12.5, -65.5, Math.toRadians(90));
    public static final Pose2d RedLeft_StartPose = new Pose2d(-35.5, -65.5, Math.toRadians(90));
    public static final Pose2d BlueRight_StartPose = new Pose2d(-35.5, 65.5, Math.toRadians(270));
    public static final Pose2d BlueLeft_StartPoseBK = new Pose2d(12.5, 65.5, Math.toRadians(270));

}
