package org.firstinspires.ftc.teamcode.common.Hardware.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;

@Config
public class Globals {

    // VELOCITY AND ACCELERATION

    public static final TrajectoryVelocityConstraint MaxVel = SampleMecanumDrive.getVelocityConstraint(
            100,
            Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint MaxAccel = SampleMecanumDrive.getAccelerationConstraint(
            100);

    public static final TrajectoryVelocityConstraint HalfVel = SampleMecanumDrive.getVelocityConstraint
            (74,
                    Math.toRadians(180), Math.toRadians(180));
    public static final TrajectoryAccelerationConstraint HalfAccel = SampleMecanumDrive.getAccelerationConstraint
            (74);

    /* ======= DIRECTIONS ======= */
    public static final double NORTH = Math.toRadians(90);
    public static final double SOUTH = Math.toRadians(270);
    public static final double EAST = Math.toRadians(0);
    public static final double WEST = Math.toRadians(180);

    /* ======= COORDINATE CONSTANTS ======= */
    /* ======= START POSE ======= */
    public static final double HEADING_RED = Math.toRadians(90);
    public static final Pose2d StartPoseRED_LEFT = new Pose2d(12, -62.5, HEADING_RED);
    public static final Pose2d StartPoseRED_RIGHT = new Pose2d(-36, -62.5, HEADING_RED);

    public static final double HEADING_BLUE = Math.toRadians(270);
    public static final Pose2d StartPoseBLUE_LEFT = new Pose2d(-36, 62.5, HEADING_BLUE);
    public static final Pose2d StartPoseBlue_RIGHT = new Pose2d(12, 62.5, HEADING_BLUE);

    /* ======= SPIKEMARK POSITIONING POSE (RED)  ======= */
    public static final Pose2d RED_SPIKEMARK_RIGHT = new Pose2d(12, -33.5);
    public static final Pose2d RED_SPIKEMARK_LEFT = new Pose2d(-36,-33.5);

    // RED RIGHT
    public static final Pose2d RED_TEAMPROP_LEFT_POSE = new Pose2d(12, -33.5, WEST);
    public static final Pose2d RED_TEAMPROP_RIGHT_POSE = new Pose2d(12, -33.5, EAST);

    /* ======= SPIKEMARK POSITIONING POSE (BLUE) ======= */
    public static final Vector2d BLUE_SPIKEMARK_RIGHT = new Vector2d(-36, 33.5);
    public static final Pose2d BLUE_SPIKEMARK_LEFT = new Pose2d(12, 33.5);

    /* ======= BACKDROP POSITIONING POSE (RED) ====== */
    public static final Pose2d RED_BACKDROP_RIGHT = new Pose2d(52, -33.5, WEST);


    /* ======= END COORDINATE CONSTANTS  ======= */


}
