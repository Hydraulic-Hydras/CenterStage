package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;

@Config
public class Globals {

    // VELOCITY AND ACCELERATION
    public static final TrajectoryVelocityConstraint MaxVel = SampleMecanumDrive.getVelocityConstraint(
            100, Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint MaxAccel = SampleMecanumDrive.getAccelerationConstraint(
            100);

    public static final TrajectoryVelocityConstraint HalfVel = SampleMecanumDrive.getVelocityConstraint(
            74, Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint HalfAccel = SampleMecanumDrive.getAccelerationConstraint
            (74);

    // Start Position
    public static final Pose2d StartPose = new Pose2d(0, 0, Math.toRadians(0));

    /* === RED RIGHT BK === */
    public static final Vector2d splineToProp = new Vector2d(19.5, -8.6);
    public static final Vector2d lineToBackDrop = new Vector2d(17, -28);

    /* === RED LEFT === */
    public static final Vector2d splineToDoor = new Vector2d(47, -17);
    public static final Pose2d reversedTransfer = new Pose2d(47, -50);
    public static final Vector2d S_splineToBackDrop = new Vector2d(22, -74);

    public static final Vector2d lineToCenterProp = new Vector2d(34, 12);
    public static final Vector2d lineToPos = new Vector2d(39, 2);
    public static final Vector2d straightLineToPos = new Vector2d(49, -55);
    public static final Vector2d curveSplineToBackDrop = new Vector2d(30, -84.2);

    /* === BLUE LEFT BK === */
    public static final Vector2d BLUE_splineToLeftProp = new Vector2d(17, 7);
    public static final Vector2d BLUE_lineToBackDrop = new Vector2d(17, 31.5);
}
