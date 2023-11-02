package org.firstinspires.ftc.teamcode.CenterStage.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.hydraulichydras.hydrauliclib.Motion.MotionProfiledDcMotor;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class Climbers extends Contraption {

    private MotionProfiledDcMotor leftClimber;
    private MotionProfiledDcMotor rightClimber;

    private TouchSensor high_Climb;
    private TouchSensor low_Climb;

    public static final double WHEEL_RADIUS = 0; // inches
    public static final double GEAR_RATIO = 0;
    public static final double TICKS_PER_REV = 0;

    public static double MAX_VEL = 100;
    public static double MAX_ACCEL = 100;
    public static final double RETRACTION_MULTIPLIER = 1;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double POS_REST = 0;
    public static double POS_EXTENDED = 0;

    public Climbers(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    @Override
    public void init(HardwareMap hwMap) {
        leftClimber = new MotionProfiledDcMotor(hwMap, "LeftClimber");
        leftClimber.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        leftClimber.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        leftClimber.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        leftClimber.setPIDCoefficients(kP, kI, kD, kF);
        leftClimber.setTargetPosition(0);

        leftClimber.setDirection(DcMotorSimple.Direction.REVERSE);

        rightClimber = new MotionProfiledDcMotor(hwMap, "RightClimber");
        rightClimber.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        rightClimber.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        rightClimber.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        rightClimber.setPIDCoefficients(kP, kI, kD, kF);
        rightClimber.setTargetPosition(0);

        rightClimber.setDirection(DcMotorSimple.Direction.REVERSE);

        low_Climb = hwMap.get(TouchSensor.class, "low_Climb");
        high_Climb = hwMap.get(TouchSensor.class, "high_Climb");
    }

    @Override
    public void loop(Gamepad gamepad1) {

        // Manual control
        if (!high_Climb.isPressed() && gamepad1.right_trigger > 0) {
            rightClimber.setPower(1);
            leftClimber.setPower(1);
        } else if (!low_Climb.isPressed() && gamepad1.left_trigger > 0) {
            rightClimber.setPower(-1);
            leftClimber.setPower(-1);
        } else {
            leftClimber.setPower(0);
            rightClimber.setPower(0);
        }

        // Auto move
        if (gamepad1.a) {
            // fully extended
            extended();
        }   else if (gamepad1.b) {
            // default
            rest();
        }

    }

    // AUTO functions
    public void setMotionConstraints(double value) {
        rightClimber.setMotionConstraints(value, value);
        leftClimber.setMotionConstraints(value, value);
    }

    public void rest() {
        rightClimber.setTargetPosition(POS_REST);
        leftClimber.setTargetPosition(POS_REST);
    }

    public void extended() {
        rightClimber.setTargetPosition(POS_EXTENDED);
        leftClimber.setTargetPosition(POS_EXTENDED);
    }

    public void setPower(double power) {
        leftClimber.setPower(power);
        rightClimber.setPower(power);
    }

    public void update() {
        leftClimber.update();
        rightClimber.update();
    }

    public double getPosition() {
        return ((leftClimber.getPosition() + rightClimber.getPosition()) / 2.0 );
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("positionLeft", leftClimber.getPosition());
        telemetry.addData("positionRight", rightClimber.getPosition());

        telemetry.addData("VelocityLeft", leftClimber.getVelocity());
        telemetry.addData("VelocityRight", rightClimber.getVelocity());

        telemetry.addData("currentLeft", leftClimber.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("currentRight", rightClimber.getCurrent(CurrentUnit.AMPS));
    }
}
