package org.firstinspires.ftc.teamcode.CenterStage.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class AutoConstantsCenterStage {

    // VELOCITY AND ACCELERATION
    // MAX

    public static final TrajectoryVelocityConstraint MAX_VEL = SampleMecanumDrive.getVelocityConstraint(100, Math.toRadians(180),
            Math.toRadians(180));

    public static final TrajectoryAccelerationConstraint MAX_ACCEL = SampleMecanumDrive.getAccelerationConstraint(100);

    // Quick response
    public static final TrajectoryVelocityConstraint FAST_VEL = SampleMecanumDrive.getVelocityConstraint(74, Math.toRadians(180),
            Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(74);

    // Normal speed
    public static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(58, Math.toRadians(180),
            Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(58);


    // BLUE RIGHT START POSE
    public static final Pose2d BR_START = new Pose2d(-36, 62, Math.toRadians(270));

    // BLUE LEFT START POSE
    public static final Pose2d BL_START = new Pose2d(12, 62, Math.toRadians(270));

    // RED RIGHT START POSE
    public static final Pose2d RR_START = new Pose2d(12, -62, Math.toRadians(180));

    // RED LEFT START POSE
    public static final Pose2d RL_START = new Pose2d(-36, -62, Math.toRadians(180));

    // WHITE STACK POSES



}
