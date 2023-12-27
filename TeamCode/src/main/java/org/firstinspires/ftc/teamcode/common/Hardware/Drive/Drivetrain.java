package org.firstinspires.ftc.teamcode.common.Hardware.Drive;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Drivetrain extends Contraption {

    public double powerMultiplier = 1;
    public SampleMecanumDrive drive;
    private DigitalChannel LED_GreenL;
    private DigitalChannel LED_RedL;
    private DigitalChannel LED_GreenR;
    private DigitalChannel LED_RedR;
    private DistanceSensor distanceBackdrop;


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
        }   else {
            powerMultiplier = 1;
        }

        drive.setMotorPowers(leftFrontSpeed * powerMultiplier, leftRearSpeed * powerMultiplier,
                rightRearSpeed * powerMultiplier, rightFrontSpeed * powerMultiplier);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Backdrop Distance", Double.parseDouble(
                JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)));
        telemetry.update();
    }
}
