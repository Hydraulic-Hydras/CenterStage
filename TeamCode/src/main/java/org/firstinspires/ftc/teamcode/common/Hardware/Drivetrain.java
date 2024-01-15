package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Drivetrain extends Contraption {

    public double powerMultiplier = 1;

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;
    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void initialize(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            powerMultiplier = 0.4;
        } else {
            powerMultiplier = 1;
        }

        leftFront.setPower(leftFrontSpeed * powerMultiplier);
        leftRear.setPower(leftRearSpeed * powerMultiplier);
        rightFront.setPower(rightFrontSpeed * powerMultiplier);
        rightRear.setPower(rightRearSpeed * powerMultiplier);

    }
}
