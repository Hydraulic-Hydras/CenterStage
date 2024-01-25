package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Climber extends Contraption {

    public static DcMotor leftHang;
    public static DcMotor rightHang;
    public static Servo Hook;
    public enum ClimberState {
        REST,
        RAMPING,
        EXTENDED
    }

    public static double HOOK_RELEASE = 0.5;
    public static double HOOK_ON = 0.7;

    ClimberState climberState = ClimberState.REST;
    public Climber(LinearOpMode opMode) {this.opMode = opMode;}
    @Override
    public void initialize(HardwareMap hardwareMap) {
        leftHang = hardwareMap.get(DcMotorEx.class, "climber-L");
        leftHang.setDirection(DcMotor.Direction.REVERSE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightHang = hardwareMap.get(DcMotorEx.class, "climber-R");
        rightHang.setDirection(DcMotor.Direction.REVERSE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Hook = hardwareMap.get(Servo.class, "hook");

        Hook.setPosition(HOOK_RELEASE);
    }

    public void loop(Gamepad gamepad2) {
        double power = gamepad2.left_stick_y;
        double speedDec = 0.4;

        leftHang.setPower(power * speedDec);
        rightHang.setPower(power * speedDec);

        if (Mitsumi.high_Limit.isPressed() && Hook.getPosition() == HOOK_ON) {
            climberState = ClimberState.EXTENDED;
        }   else if (!Mitsumi.high_Limit.isPressed()) {
            climberState = ClimberState.RAMPING;
        }   else if (Mitsumi.low_Limit.isPressed()) {
            climberState = ClimberState.REST;
        }


        if (gamepad2.right_bumper) {
            leftHang.setPower(0.3);
            rightHang.setPower(0.3);
        }   else {
            leftHang.setPower(0);
            rightHang.setPower(0);
        }

        if (gamepad2.dpad_up) {
            Hook.setPosition(HOOK_RELEASE);
        }   else if (gamepad2.dpad_down) {
            Hook.setPosition(HOOK_ON);
        }

    }
}
