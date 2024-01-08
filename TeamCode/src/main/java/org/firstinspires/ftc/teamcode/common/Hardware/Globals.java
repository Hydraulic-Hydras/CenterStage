package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
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

}
