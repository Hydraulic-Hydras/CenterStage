package org.firstinspires.ftc.teamcode.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class AutoConstants {

    // VELOCITY AND ACCELERATION
    public static final int MAX_VEL = 100;
    public static final int MAX_ACCEL = 100;
    public static final int FAST_VEL = 74;
    public static final int FAST_ACCEL = 74;
    public static final int VELO = 58;
    public static final int ACCEL = 58;

    public static final TrajectoryVelocityConstraint Vel0 = SampleMecanumDrive.getVelocityConstraint
            (FAST_VEL,
            Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint Accel0 = SampleMecanumDrive.getAccelerationConstraint
            (FAST_ACCEL);


    /* ======= COORDINATE CONSTANTS ======= */
    public static final double HEADING = Math.toRadians(90);
    public static final double WALLPOS = -1 * (70.5 - (16.5 / 2.0));
    public static final int LEFT_X = -35;
    public static final Pose2d LEFT_SP = new Pose2d(LEFT_X, WALLPOS, HEADING);
    public static final int RIGHT_X = 12;
    public static final Pose2d RIGHT_SP = new Pose2d(RIGHT_X, WALLPOS, HEADING);

        /* === LEFT SIDE === */
    public static final int PROP_UNLOAD_X = -35;
    public static final int PROP_UNLOAD_Y = -32;
    public static final double PROP_UNLOAD_HEADING_LEFT = Math.toRadians(180);
    public static final Pose2d LEFT_PROP_UNLOAD_POS = new Pose2d(PROP_UNLOAD_X, PROP_UNLOAD_Y, PROP_UNLOAD_HEADING_LEFT);
    public static final double PROP_UNLOAD_HEADING_RIGHT = Math.toRadians(0);
    public static final Pose2d RIGHT_PROP_UNLOAD_POS = new Pose2d(PROP_UNLOAD_X, PROP_UNLOAD_Y, PROP_UNLOAD_HEADING_RIGHT);
    public static final int L_HIGH_PIXEL_X = -59;
    public static final double L_HIGH_PIXEL_Y = -11.60;
    public static final double HEADING_HIGH_PIXEL_LEFT = Math.toRadians(180);
    public static final Vector2d UPPER_WHITE_STACK = new Vector2d(L_HIGH_PIXEL_X, L_HIGH_PIXEL_Y);
    public static final int CENTER_DOOR_POS_X = 20;
    public static final double CENTER_DOOR_POS_Y = -11.60;
    public static final Vector2d CENTER_DOOR_DELAYPOS = new Vector2d(CENTER_DOOR_POS_X, CENTER_DOOR_POS_Y);
    public static final int BACKDROP_LEFT_X = 48;
    public static final int BACKDROP_LEFT_Y = -29;
    public static final Vector2d BACKDROP_LEFT_SCORE = new Vector2d(BACKDROP_LEFT_X, BACKDROP_LEFT_Y);
    public static final double HEADING_LEFT_BACKDROP = Math.toRadians(0);
    public static final double HEADING_LEFT_REVERT_BACKDROP = Math.toRadians(180);

    /* ======= END COORDINATE CONSTANTS  ======= */
}