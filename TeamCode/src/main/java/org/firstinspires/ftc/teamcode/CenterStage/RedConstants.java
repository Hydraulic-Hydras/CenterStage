package org.firstinspires.ftc.teamcode.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class RedConstants {

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
    public static final double HEADING = Math.toRadians(90);
    public static final double WALLPOS = -1 * (70.5 - (16.5 / 2.0));
    public static final int LEFT_X = -35;
    public static final Pose2d LEFT_SP = new Pose2d(LEFT_X, WALLPOS, HEADING);
    public static final int RIGHT_X = 12;
    public static final Pose2d RIGHT_SP = new Pose2d(RIGHT_X, WALLPOS, HEADING);

        /* === LEFT SIDE === */
    public static final int L_PROP_UNLOAD_X = -35;
    public static final int L_PROP_UNLOAD_Y = -32;
    public static final double L_PROP_UNLOAD_HEADING_LEFT = Math.toRadians(180);
    public static final Pose2d L_LEFT_PROP_UNLOAD_POSE = new Pose2d(L_PROP_UNLOAD_X, L_PROP_UNLOAD_Y, L_PROP_UNLOAD_HEADING_LEFT);
    public static final double L_PROP_UNLOAD_HEADING_RIGHT = Math.toRadians(0);
    public static final Pose2d L_RIGHT_PROP_UNLOAD_POS = new Pose2d(L_PROP_UNLOAD_X, L_PROP_UNLOAD_Y, L_PROP_UNLOAD_HEADING_RIGHT);

    // LEFT TRAJECTORY
    public static final int L_HIGH_PIXEL_X = -59;
    public static final double L_HIGH_PIXEL_Y = -11.60;
    public static final double HEADING_HIGH_PIXEL = Math.toRadians(180);
    public static final Vector2d UPPER_WHITE_STACK_VECTOR = new Vector2d(L_HIGH_PIXEL_X, L_HIGH_PIXEL_Y);
    public static final int CENTER_DOOR_POS_X = 20;
    public static final double CENTER_DOOR_POS_Y = -11.60;
    public static final Vector2d CENTER_DOOR_DELAYPOS_VECTOR = new Vector2d(CENTER_DOOR_POS_X, CENTER_DOOR_POS_Y);
    public static final double HEADING_LEFT_BACKDROP = Math.toRadians(0);
    public static final double HEADING_LEFT_REVERT_BACKDROP = Math.toRadians(180);

    // TEAM PROP LOCATION == LEFT BACKDROP
    public static final int L_BACKDROP_LEFT_X = 48;
    public static final int L_BACKDROP_LEFT_Y = -29;
    public static final Vector2d L_BACKDROP_LEFT_SCORE = new Vector2d(L_BACKDROP_LEFT_X, L_BACKDROP_LEFT_Y);

    // TEAM PROP LOCATION == MIDDLE BACKDROP
    public static final int L_BACKDROP_MIDDLE_X = 48;
    public static final int L_BACKDROP_MIDDLE_Y = -35;
    public static final Vector2d L_BACKDROP_MIDDLE_SCORE = new Vector2d(L_BACKDROP_MIDDLE_X, L_BACKDROP_MIDDLE_Y);

    // TEAM PROP LOCATION == RIGHT BACKDROP
    public static final int L_BACKDROP_RIGHT_X = 48;
    public static final int L_BACKDROP_RIGHT_Y = -42;
    public static final Vector2d L_BACKDROP_RIGHT_SCORE = new Vector2d(L_BACKDROP_RIGHT_X, L_BACKDROP_RIGHT_Y);

    /* === RIGHT SIDE === */
    public static final int R_PROP_UNLOAD_X = 12;
    public static final int R_PROP_UNLOAD_Y = -29;
    public static final double R_PROP_UNLOAD_HEADING_LEFT = Math.toRadians(180);
    public static final Pose2d R_LEFT_PROP_UNLOAD_POS = new Pose2d(R_PROP_UNLOAD_X, R_PROP_UNLOAD_Y, R_PROP_UNLOAD_HEADING_LEFT);
    public static final double R_PROP_UNLOAD_HEADING_RIGHT = Math.toRadians(0);
    public static final Pose2d R_RIGHT_PROP_UNLOAD_POS = new Pose2d(R_PROP_UNLOAD_X, R_PROP_UNLOAD_Y, R_PROP_UNLOAD_HEADING_RIGHT);

    // RIGHT TRAJECTORY
    // take in account that "UNLOAD" for RIGHT is also positioning
    public static final int R_TAPE_POS_X = 35;
    public static final double R_TAPE_POS_Y = -58.5;
    public static final Vector2d R_TAPE_POS_VECTOR = new Vector2d(R_TAPE_POS_X, R_TAPE_POS_Y);
    public static final double R_TAPE_POS_HEADING = Math.toRadians(180);
    public static final int R_BLUE_STRIPE_X = -35;
    public static final double R_BLUE_STRIPE_Y = -58.5;
    public static final Vector2d R_BLUE_STRIPE_VECTOR = new Vector2d(R_BLUE_STRIPE_X, R_BLUE_STRIPE_Y);
    public static final int R_LOWER_PIXEL_X = -58;
    public static final double R_LOWER_PIXEL_Y = -35.45;
    public static final double R_LOWER_PIXEL_HEADING = Math.toRadians(180);
    public static final Vector2d LOWER_PIXEL_VECTOR = new Vector2d(R_LOWER_PIXEL_X, R_LOWER_PIXEL_Y);
    public static final double HEADING_RIGHT_REVERT_BLUE_STRIPE = Math.toRadians(0);

    // TEAM PROP LOCATION == LEFT BACKDROP
    public static final int R_BACKDROP_LEFT_X = 48;
    public static final int R_BACKDROP_LEFT_Y = -29;
    public static final Vector2d R_BACKDROP_LEFT_SCORE = new Vector2d(R_BACKDROP_LEFT_X, R_BACKDROP_LEFT_Y);

    // TEAM PROP LOCATION == MIDDLE BACKDROP
    public static final int R_BACKDROP_MIDDLE_X = 48;
    public static final double R_BACKDROP_MIDDLE_Y = -35.2;
    public static final Vector2d R_BACKDROP_MIDDLE_SCORE = new Vector2d(R_BACKDROP_MIDDLE_X, R_BACKDROP_MIDDLE_Y);

    // TEAM PROP LOCATION == RIGHT BACKDROP
    public static final int R_BACKDROP_RIGHT_X = 48;
    public static final int R_BACKDROP_RIGHT_Y = -42;
    public static final Vector2d R_BACKDROP_RIGHT_SCORE = new Vector2d(R_BACKDROP_RIGHT_X, R_BACKDROP_RIGHT_Y);

    /* ======= END COORDINATE CONSTANTS  ======= */
}