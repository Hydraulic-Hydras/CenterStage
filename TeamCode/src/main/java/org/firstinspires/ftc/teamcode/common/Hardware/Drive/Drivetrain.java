package org.firstinspires.ftc.teamcode.common.Hardware.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;

@Config
public class Drivetrain extends Contraption {

    public double powerMultiplier = 1;
    public SampleMecanumDrive drive;

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    @Override
    public void initialize(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
    }

    public void RobotCentric(Gamepad gamepad1) {
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
}
