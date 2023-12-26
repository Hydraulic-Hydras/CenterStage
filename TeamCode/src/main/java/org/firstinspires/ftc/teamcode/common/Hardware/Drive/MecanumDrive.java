package org.firstinspires.ftc.teamcode.common.Hardware.Drive;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Geometry.Vector2D;
import com.hydraulichydras.hydrauliclib.Localization.Drivetrain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Util.HSubsystem;
import org.firstinspires.ftc.teamcode.Tuning.DriveConstants;

import java.util.Arrays;

public class MecanumDrive extends HSubsystem implements Drivetrain {

    IMU imu;
    public double powerMultiplier = 1;
    private final RobotHardware robot = RobotHardware.getInstance();
    double[] ws = new double[4];
    // auto power i think
    double[] wws = new double[4];
    public MecanumDrive() {}

    // only use if robot has to be in field centric mode
    public void initGyro(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
    }

    @Override
    public void set(Pose pose) {
        set(pose, 0);
    }


    public void RobotCentric(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double [] wheelspeeds = new double[4];

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontSpeed = (y + x + rx) / denominator;
        double leftRearSpeed = (y - x + rx) / denominator;
        double rightFrontSpeed = (y - x - rx) / denominator;
        double rightRearSpeed = (y + x - rx) / denominator;

        if (gamepad1.left_bumper) {
            powerMultiplier = 0.5;
        }   else {
            powerMultiplier = 1;
        }

        wheelspeeds[0] = leftFrontSpeed * powerMultiplier;
        wheelspeeds[1] = leftRearSpeed * powerMultiplier;
        wheelspeeds[2] = rightFrontSpeed * powerMultiplier;
        wheelspeeds[3] = rightRearSpeed * powerMultiplier;

        ws[0] = wheelspeeds[0]; // left front
        ws[1] = wheelspeeds[1]; // left rear
        ws[2] = wheelspeeds[2]; // right front
        ws[3] = wheelspeeds[3]; // right rear

    }

    public void FieldCentric(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double [] wheelspeeds = new double[4];

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontSpeed = (rotY + rotX + rx) / denominator;
        double leftRearSpeed = (rotY - rotX + rx) / denominator;
        double rightFrontSpeed = (rotY - rotX - rx) / denominator;
        double rightRearSpeed = (rotY + rotX - rx) / denominator;

        if (gamepad1.left_bumper) {
            powerMultiplier = 0.5;
        } else {
            powerMultiplier = 1;
        }

        wheelspeeds[0] = leftFrontSpeed * powerMultiplier;
        wheelspeeds[1] = leftRearSpeed * powerMultiplier;
        wheelspeeds[2] = rightFrontSpeed * powerMultiplier;
        wheelspeeds[3] = rightRearSpeed * powerMultiplier;

        ws[0] = wheelspeeds[0]; // left front
        ws[1] = wheelspeeds[1]; // left rear
        ws[2] = wheelspeeds[2]; // right front
        ws[3] = wheelspeeds[3]; // right rear
    }

    public void set(double strafeSpeed, double forwardSpeed,
                    double turnSpeed, double gyroAngle) {

        Vector2D input = new Vector2D(strafeSpeed, forwardSpeed).rotate(-gyroAngle);

        strafeSpeed = input.x;
        forwardSpeed = input.y;

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = forwardSpeed + strafeSpeed + turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = forwardSpeed - strafeSpeed - turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = (forwardSpeed - strafeSpeed + turnSpeed);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = (forwardSpeed + strafeSpeed - turnSpeed);
        // 1.06, 1.04

        double max = Arrays.stream(wheelSpeeds).max().getAsDouble();

        if (Math.abs(max) > 1) {
            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] /= max;
        }

        wws[0] = wheelSpeeds[0];
        wws[1] = wheelSpeeds[1];
        wws[2] = wheelSpeeds[2];
        wws[3] = wheelSpeeds[3];
    }

    @Override
    public void periodic() {
        // leave blank
    }

    @Override
    public void read() {
        // leave blank
    }

    @Override
    public void write() {
        robot.leftFront.setPower(wws[0]);
        robot.leftRear.setPower(wws[1]);
        robot.rightFront.setPower(wws[2]);
        robot.rightRear.setPower(wws[3]);
    }

    @Override
    public void reset() {

    }
    public void set(Pose pose, double angle) {
        set(pose.x, pose.y, pose.heading, angle);
    }
}
