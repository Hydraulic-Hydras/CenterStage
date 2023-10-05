package org.firstinspires.ftc.teamcode.CenterStage.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class KJ_Teleop extends LinearOpMode {

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    double frontLeftMotorSpeed;
    double frontRightMotorSpeed;
    double backLeftMotorSpeed;
    double backRightMotorSpeed;

    @Override
    public void runOpMode() {

        frontLeftMotor   = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor  = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor    = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor   = hardwareMap.get(DcMotor.class, "BR");

        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set ZERO POWER BEHAVIOR
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                double horizontal = 0; // x-axis movement
                double vertical = 0; // y-axis movement
                double spin = 0; // rotational movement

                double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(spin), 1);
                horizontal = gamepad1.left_stick_x;
                vertical = -gamepad1.left_stick_y;
                spin = gamepad1.right_stick_x;

                frontLeftMotorSpeed = (vertical + horizontal + spin) / denominator;
                frontRightMotorSpeed = (vertical - horizontal - spin) / denominator;
                backLeftMotorSpeed = (vertical - horizontal + spin) / denominator;
                backRightMotorSpeed = (vertical + horizontal - spin) / denominator;

                frontLeftMotor.setPower(frontLeftMotorSpeed);
                frontRightMotor.setPower(frontRightMotorSpeed);
                backLeftMotor.setPower(backLeftMotorSpeed);
                backRightMotor.setPower(backRightMotorSpeed);

            }
        }
    }
}
