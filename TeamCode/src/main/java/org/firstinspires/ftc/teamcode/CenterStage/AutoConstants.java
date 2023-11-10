package org.firstinspires.ftc.teamcode.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class AutoConstants {

     /**
      public static int LEFTSIDE_REGION_X = 0;
      public static int LEFTSIDE_REGION_Y = 170;

      public static int RIGHTSIDE_REGION_X = 0;
      public static int RIGHTSIDE_REGION_Y = 170;

      public static int REGION_WIDTH = 65;
      public static int REGION_HEIGHT = 65;
      **/

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
    public static final int LEFT_X = -35;
    public static final double L_WALLPOS = -1 * (70.5 - (16.5 / 2.0));
    public static final Pose2d LEFT_SP = new Pose2d(LEFT_X, L_WALLPOS, HEADING);
    public static final int RIGHT_X = 12;
    public static final double R_WALLPOS = 1 * (70.5 - (16.5 / 2.0));
    public static final Pose2d RIGHT_SP = new Pose2d(RIGHT_X, R_WALLPOS, HEADING);

    /* ======= END COORDINATE CONSTANTS  ======= */
}