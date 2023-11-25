package org.firstinspires.ftc.teamcode.common.Hardware.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;

@Config
public class BlueConstants {

    // VELOCITY AND ACCELERATION
    public static final int MAX_VEL = 100;
    public static final int MAX_ACCEL = 100;
    public static final int FAST_VEL = 74;
    public static final int FAST_ACCEL = 74;
    public static final int VELO = 45;
    public static final int ACCEL = 45;

    public static final TrajectoryVelocityConstraint Vel0 = SampleMecanumDrive.getVelocityConstraint
            (FAST_VEL,
                    Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint Accel0 = SampleMecanumDrive.getAccelerationConstraint
            (FAST_ACCEL);

    public static final TrajectoryVelocityConstraint SlowVel = SampleMecanumDrive.getVelocityConstraint(
            VELO,
            Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint SlowAccel = SampleMecanumDrive.getAccelerationConstraint(
            ACCEL);


    /* ======= COORDINATE CONSTANTS ======= */

        /* === LEFT SIDE === */

    // LEFT TRAJECTORY
    // take in account that "UNLOAD" for LEFT is also positioning
    // its inversed for red and blue yea i know my brain isnt braining with these poses n sh1t

    // TEAM PROP LOCATION == LEFT BACKDROP

    // TEAM PROP LOCATION == MIDDLE BACKDROP

    // TEAM PROP LOCATION == RIGHT BACKDROP

        /* === RIGHT SIDE === */

    // RIGHT TRAJECTORY

    // TEAM PROP LOCATION == LEFT BACKDROP

    // TEAM PROP LOCATION == MIDDLE BACKDROP

    // TEAM PROP LOCATION == RIGHT BACKDROP

    /* ======= END COORDINATE CONSTANTS  ======= */
}
