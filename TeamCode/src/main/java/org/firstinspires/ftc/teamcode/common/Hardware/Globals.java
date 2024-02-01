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

    // VELOCITY AND ACCELERATION
    public static final TrajectoryVelocityConstraint MaxVel = SampleMecanumDrive.getVelocityConstraint(
            100, Math.toRadians(229), Math.toRadians(229));
    public static final TrajectoryAccelerationConstraint MaxAccel = SampleMecanumDrive.getAccelerationConstraint(
            100);

    public static final TrajectoryVelocityConstraint HalfVel = SampleMecanumDrive.getVelocityConstraint(
            74, Math.toRadians(229), Math.toRadians(229));
    public static final TrajectoryAccelerationConstraint HalfAccel = SampleMecanumDrive.getAccelerationConstraint
            (74);

    // Start Position
    public static final Pose2d StartPose = new Pose2d(0, 0, Math.toRadians(0));

}
