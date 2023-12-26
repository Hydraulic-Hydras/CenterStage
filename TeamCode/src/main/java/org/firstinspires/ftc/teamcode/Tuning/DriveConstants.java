package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Config
public class DriveConstants {

    // How do you convert ticks to inches?
    // So each tick is theoretically pi/90 inches. Just multiply your encoder value by pi/90.
    public static final double TICKS_PER_REV = 420;
    public static final double MAX_RPM = 400;
    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false; // keep this false if using odometry
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
           getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    // Realizing the limitations of the internal PIDF controller is very important in keeping your sanity.

    /* NOTES
     * If the Vel0 line is overshooting, lower the kF, if it undershoots increase the kF
     * Slowly increase kP to try and get the slopes to match the target.
     * Increase kD to try and dampen any oscillations. Only minor adjustments, increasing too far
     * will increase oscillations.
     * If jittering occurs then loosen up on the tuning and use TRANSLATIONAL_PID to fix it.
     *
     * If it still overshoots or tweaking variables doesn't change anything, try lowering MAX_VEL
     * and test it again to see if that changes anything.
     */

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 15.9; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV =  0.0141; // / rpmToVelocity(MAX_RPM);
    public static double kA = 0.003;
    public static double kStatic = 0.0032;

    // kV = 0.011;
    // kA = 0.002;
    // kstatic = 0.105

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 45; // 65
    public static double MAX_ACCEL = 45; // 65
    public static double MAX_ANG_VEL = Math.toRadians( 270 );
    public static double MAX_ANG_ACCEL = Math.toRadians( 270 );

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
