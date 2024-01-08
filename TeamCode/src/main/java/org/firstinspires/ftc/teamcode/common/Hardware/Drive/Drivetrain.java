package org.firstinspires.ftc.teamcode.common.Hardware.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;

@Config
public class Drivetrain extends Contraption {

    public double powerMultiplier = 1;
    public static SampleMecanumDrive drive;
    private static DigitalChannel LED_GreenL;
    private static DigitalChannel LED_RedL;
    private static DigitalChannel LED_GreenR;
    private static DigitalChannel LED_RedR;
    public static DistanceSensor distanceBackdrop;

    private DigitalChannel ALED_Green;
    private DigitalChannel ALED_Red;
    private DigitalChannel BLED_Green;
    private DigitalChannel BLED_Red;

    public static int count = 0;

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    @Override
    public void initialize(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);

        LED_GreenL = hwMap.get(DigitalChannel.class, "LED_Green-L");
        LED_RedL = hwMap.get(DigitalChannel.class, "LED_Red-L");
        LED_GreenR = hwMap.get(DigitalChannel.class, "LED_Green-R");
        LED_RedR = hwMap.get(DigitalChannel.class, "LED_Red-R");

        distanceBackdrop = hwMap.get(DistanceSensor.class, "distanceBackdrop");

        LED_GreenL.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RedL.setMode(DigitalChannel.Mode.OUTPUT);
        LED_GreenR.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RedR.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Green = hwMap.get(DigitalChannel.class, "ALED_Green");
        ALED_Red = hwMap.get(DigitalChannel.class, "ALED_Red");
        BLED_Green = hwMap.get(DigitalChannel.class, "BLED_Green");
        BLED_Red = hwMap.get(DigitalChannel.class, "BLED_Red");

        ALED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        ALED_Red.setMode(DigitalChannel.Mode.OUTPUT);
        BLED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        BLED_Red.setMode(DigitalChannel.Mode.OUTPUT);


    }

    public void RobotCentric(Gamepad gamepad1) {

        if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) >= 16
                && Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 22) {
            LED_RedL.setState(false);
            LED_GreenL.setState(true);
            LED_RedR.setState(false);
            LED_GreenR.setState(true);
        } else {
            LED_GreenL.setState(false);
            LED_RedL.setState(true);
            LED_GreenR.setState(false);
            LED_RedR.setState(true);
        }

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

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
        } else {
            powerMultiplier = 1;
        }

        drive.setMotorPowers(leftFrontSpeed * powerMultiplier, leftRearSpeed * powerMultiplier,
                rightRearSpeed * powerMultiplier, rightFrontSpeed * powerMultiplier);

        if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) >= 16 && Double.parseDouble(
                JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 22
                    && Intake.rotateBucket.getPosition() != 0.2) {
            ALED_Green.setState(true);
            ALED_Red.setState(false);
            BLED_Green.setState(true);
            BLED_Red.setState(false);
        } else if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 40) {
            ALED_Green.setState(false);
            ALED_Red.setState(true);
            BLED_Green.setState(false);
            BLED_Red.setState(true);
        } else if (Launcher.launcher_angle.getPosition() < 0.4) {
            if (Launcher.droneTrigger.getPosition() > 0.5) {
                ALED_Green.setState(true);
                ALED_Red.setState(false);
                BLED_Green.setState(true);
                BLED_Red.setState(false);
            } else if (count == 1) {
                ALED_Green.setState(false);
                ALED_Red.setState(true);
                BLED_Green.setState(false);
                BLED_Red.setState(true);
                count = 0;
            } else {
                ALED_Green.setState(true);
                ALED_Red.setState(true);
                BLED_Green.setState(true);
                BLED_Red.setState(true);
                count = 1;
            }
        } else {
            ALED_Green.setState(true);
            ALED_Red.setState(true);
            BLED_Green.setState(true);
            BLED_Red.setState(true);
        }
    }

    public void telemetry(Telemetry telemetry) {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        telemetry.addData("Backdrop Distance", Double.parseDouble(
                JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)));
        telemetry.update();
    }
}
