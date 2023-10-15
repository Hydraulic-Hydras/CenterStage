package org.firstinspires.ftc.teamcode.CenterStage.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoConstantsPowerplay {

    // VELOCITY && ACCELERATION
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

    // RED RIGHT START POSE
    public static final int RR_X = 35;
    public static final int RR_Y = -72;
    public static final double RR_H = Math.toRadians(90);
    public static final Pose2d RR_START = new Pose2d(RR_X, RR_Y, RR_H);

    // testing space
    public static final Pose2d RR_JUNCTION = new Pose2d(35,0,Math.toRadians(90));

    // RED RIGHT PARK == middle
    public static final int RR_PARK_MIDDLE_X = 34;
    public static final int RR_PARK_MIDDLE_Y = -18;
    public static final double RR_PARK_MIDDLE_H = Math.toRadians(90);

    public static final Pose2d RR_MIDDLE_PARK = new Pose2d(RR_PARK_MIDDLE_X, RR_PARK_MIDDLE_Y, RR_PARK_MIDDLE_H);

    // RED RIGHT PARK == left
    public static final int RR_PARK_LEFT_X = 10;
    public static final int RR_PARK_LEFT_Y = -18;
    public static final double RR_PARK_LEFT_H = Math.toRadians(90);

    public static final Pose2d RR_LEFT_PARK = new Pose2d(RR_PARK_LEFT_X, RR_PARK_LEFT_Y, RR_PARK_LEFT_H);

    // RED RIGHT PARK == right
    public static final int RR_PARK_RIGHT_X = 60;
    public static final int RR_PARK_RIGHT_Y = -18;
    public static final double RR_PARK_RIGHT_H = Math.toRadians(90);

    public static final Pose2d RR_RIGHT_PARK = new Pose2d(RR_PARK_RIGHT_X, RR_PARK_RIGHT_Y, RR_PARK_RIGHT_H);





    // RED LEFT START POSE
    public static final int RL_X = -35;
    public static final int RL_Y = -72;
    public static final double RL_H = Math.toRadians(90);
    public static final Pose2d RL_START = new Pose2d(RL_X, RL_Y, RL_H);

    // RED LEFT PARK == middle
    public static final int RL_PARK_MIDDLE_X = -34;
    public static final int RL_PARK_MIDDLE_Y = -18;
    public static final double RL_PARK_MIDDLE_H = Math.toRadians(90);

    public static final Pose2d RL_MIDDLE_PARK = new Pose2d(RL_PARK_MIDDLE_X, RL_PARK_MIDDLE_Y, RL_PARK_MIDDLE_H);

    // RED LEFT PARK == left
    public static final int RL_PARK_LEFT_X = -60;
    public static final int RL_PARK_LEFT_Y = -18;
    public static final double RL_PARK_LEFT_H = Math.toRadians(90);

    public static final Pose2d RL_LEFT_PARK = new Pose2d(RL_PARK_LEFT_X, RL_PARK_LEFT_Y, RL_PARK_LEFT_H);

    // RED LEFT PARK == right
    public static final int RL_PARK_RIGHT_X = -10;
    public static final int RL_PARK_RIGHT_Y = -18;
    public static final double RL_PARK_RIGHT_H = Math.toRadians(90);

    public static final Pose2d RL_RIGHT_PARK = new Pose2d(RL_PARK_RIGHT_X, RL_PARK_RIGHT_Y, RL_PARK_RIGHT_H);



    // BLUE LEFT START POSE
    public static final int BL_X = 35;
    public static final int BL_Y = 72;
    public static final double BL_H = Math.toRadians(270);
    public static final Pose2d BL_START = new Pose2d(BL_X, BL_Y, BL_H);

    // BLUE LEFT PARK == middle
    public static final int BL_PARK_MIDDLE_X = 34;
    public static final int BL_PARK_MIDDLE_Y = 18;
    public static final double BL_PARK_MIDDLE_H = Math.toRadians(270);
    public static final Pose2d BL_MIDDLE_PARK = new Pose2d(BL_PARK_MIDDLE_X, BL_PARK_MIDDLE_Y, BL_PARK_MIDDLE_H);

    // BLUE LEFT PARK == left
    public static final int BL_PARK_LEFT_X = 60;
    public static final int BL_PARK_LEFT_Y = 18;
    public static final double BL_PARK_LEFT_H = Math.toRadians(270);
    public static final Pose2d BL_LEFT_PARK = new Pose2d(BL_PARK_LEFT_X, BL_PARK_LEFT_Y, BL_PARK_LEFT_H);

    // BLUE LEFT PARK == right
    public static final int BL_PARK_RIGHT_X = 10;
    public static final int BL_PARK_RIGHT_Y = 18;
    public static final double BL_PARK_RIGHT_H = Math.toRadians(270);
    public static final Pose2d BL_RIGHT_PARK = new Pose2d(BL_PARK_RIGHT_X, BL_PARK_RIGHT_Y, BL_PARK_RIGHT_H);



    // BLUE LEFT START POSE
    public static final int BR_X = -35;
    public static final int BR_Y = 72;
    public static final double BR_H = Math.toRadians(270);
    public static final Pose2d BR_START = new Pose2d(BR_X, BR_Y, BR_H);

    // BLUE RIGHT PARK == middle
    public static final int BR_PARK_MIDDLE_X = -34;
    public static final int BR_PARK_MIDDLE_Y = 18;
    public static final double BR_PARK_MIDDLE_H = Math.toRadians(270);

    public static final Pose2d BR_MIDDLE_PARK = new Pose2d(BR_PARK_MIDDLE_X, BR_PARK_MIDDLE_Y, BR_PARK_MIDDLE_H);

    // BLUE RIGHT PARK == right
    public static final int BR_PARK_RIGHT_X = -60;
    public static final int BR_PARK_RIGHT_Y = 18;
    public static final double BR_PARK_RIGHT_H = Math.toRadians(270);

    public static final Pose2d BR_RIGHT_PARK = new Pose2d(BR_PARK_RIGHT_X, BR_PARK_RIGHT_Y, BR_PARK_RIGHT_H);

    // BLUE RIGHT PARK == left
    public static final int BR_PARK_LEFT_X = -10;
    public static final int BR_PARK_LEFT_Y = 18;
    public static final double BR_PARK_LEFT_H = Math.toRadians(270);

    public static final Pose2d BR_LEFT_PARK = new Pose2d(BR_PARK_LEFT_X, BR_PARK_LEFT_Y, BR_PARK_LEFT_H);

}
