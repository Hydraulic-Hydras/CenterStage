package com.hydraulichydras.hydralib.Kinematics;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

import java.util.function.DoubleSupplier;

/**
 * Can also be referred to as 2-Wheel Odometry
 */
public class DifferentialOdometry extends Odometry {

    private double prevLeftEncoder, prevRightEncoder;
    private Rotation2d previousAngle;

    // the suppliers
    DoubleSupplier m_left, m_right;

    public DifferentialOdometry(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                                double trackwidth) {
        this(trackwidth);
        m_left = leftEncoder;
        m_right = rightEncoder;
    }

    /**
     * The constructor that specifies the track width of the robot but defaults
     * the position to (0, 0, 0).
     *
     * @param trackWidth the track width of the robot in inches
     */

    public DifferentialOdometry(double trackWidth) {
        this(new Pose2d(), trackWidth);
    }

    /**
     * The constructor that specifies the starting position and the track width
     *
     * @param initialPose the starting position of the robot
     * @param trackWidth the track width of your robot
     */
    public DifferentialOdometry(Pose2d initialPose, double trackWidth) {
        super(initialPose, trackWidth);
        previousAngle = initialPose.getRotation();
    }

    /**
     * Updates the position of the robot
     *
     * @param newPose the new {@link Pose2d}
     */

    @Override
    public void updatePose(Pose2d newPose) {
        previousAngle = newPose.getRotation();
        robotPose = newPose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
    }

    /**
     * This does everything for you
     */

    @Override
    public void updatePose() {
        updatePosition(
                m_right.getAsDouble(),
                m_left.getAsDouble()
        );
    }


    /**
     * @param leftEncoderPos  the encoder position of the left encoder.
     * @param rightEncoderPos the encoder position of the right encoder.
     */
    public void updatePosition(double leftEncoderPos, double rightEncoderPos) {
        double deltaLeftDistance = leftEncoderPos - prevLeftEncoder;
        double deltaRightDistance = rightEncoderPos - prevRightEncoder;

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;

        double dx = (deltaLeftDistance + deltaRightDistance) / 2.0;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeftDistance - deltaRightDistance) / trackwidth
                )
        );

        Pose2d newPose = robotPose.exp(
                new Twist2d(dx, 0.0, angle.minus(previousAngle).getRadians())
        );

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
}