package org.firstinspires.ftc.teamcode.CenterStage.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Motion.MotionProfiledDcMotor;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Cascades extends Contraption {

    private MotionProfiledDcMotor leftcascade;
    private MotionProfiledDcMotor rightcascade;

    private TouchSensor high_lift;
    private TouchSensor low_lift;

    public static final double WHEEL_RADIUS = 0; // inches
    public static final double GEAR_RATIO = 1;
    private static final double TICKS_PER_REV = 0;

    public static double MAX_VEL = 100;
    public static double MAX_ACCEL = 100;
    public static double RETRACTION_MULTIPLIER = 1;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double POS_REST = 0;
    public static double POS_GROUND = 0;
    public static double POS_LOW = 0;
    public static double POS_MEDIUM = 0;
    public static double POS_HIGH = 0;
    public static double POS_HIGH_AUTO = 0;

    public Cascades(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void initialize(HardwareMap hwMap) {
        leftcascade = new MotionProfiledDcMotor(hwMap, "leftCascade");
        leftcascade.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        leftcascade.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        leftcascade.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        leftcascade.setPIDCoefficients(kP, kI, kD, kF);
        leftcascade.setTargetPosition(0);

        leftcascade.setDirection(DcMotorSimple.Direction.REVERSE);

        rightcascade = new MotionProfiledDcMotor(hwMap, "rightCascade");
        rightcascade.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        rightcascade.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        rightcascade.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        rightcascade.setPIDCoefficients(kP, kI, kD, kF);
        rightcascade.setTargetPosition(0);

        rightcascade.setDirection(DcMotorSimple.Direction.REVERSE);

        high_lift = hwMap.get(TouchSensor.class, "high_Limit");
        low_lift = hwMap.get(TouchSensor.class, "low_Limit");
    }

    @Override
    public void loop(Gamepad gamepad2) {

        if (!high_lift.isPressed() && gamepad2.right_trigger > 0) {
            rightcascade.setPower(1);
            leftcascade.setPower(1);
        } else if (!low_lift.isPressed() && gamepad2.left_trigger > 0) {
            rightcascade.setPower(-1);
            leftcascade.setPower(-1);
        } else {
            leftcascade.setPower(0);
            rightcascade.setPower(0);
        }
    }


    // AUTO functions
    public void setMotionConstraints(double value) {
        leftcascade.setMotionConstraints(value, value);
        rightcascade.setMotionConstraints(value, value);
    }

    public void rest() {
        leftcascade.setTargetPosition(POS_REST);
        rightcascade.setTargetPosition(POS_REST);
    }

    public void extendGround() {
        leftcascade.setTargetPosition(POS_GROUND);
        rightcascade.setTargetPosition(POS_GROUND);
    }

    public void extendLow() {
        leftcascade.setTargetPosition(POS_LOW);
        rightcascade.setTargetPosition(POS_LOW);
    }

    public void extendMedium() {
        leftcascade.setTargetPosition(POS_MEDIUM);
        rightcascade.setTargetPosition(POS_MEDIUM);
    }

    public void extendHigh() {
        leftcascade.setTargetPosition(POS_HIGH);
        rightcascade.setTargetPosition(POS_HIGH);
    }

    public void extendHighAuto() {
        leftcascade.setTargetPosition(POS_HIGH_AUTO);
        rightcascade.setTargetPosition(POS_HIGH_AUTO);
    }

    public void extendToPosition(double pos) {
        leftcascade.setTargetPosition(pos);
        rightcascade.setTargetPosition(pos);
    }

    public void setPower(double power) {
        leftcascade.setPower(power);
        rightcascade.setPower(power);
    }

    public void update() {
        leftcascade.update();
        rightcascade.update();
    }

    public double getPosition() {
        return (leftcascade.getPosition() + rightcascade.getPosition()) / 2.0;
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("positionLeft", leftcascade.getPosition());
        telemetry.addData("positionRight", rightcascade.getPosition());

        telemetry.addData("VelocityLeft", leftcascade.getVelocity());
        telemetry.addData("VelocityRight", rightcascade. getVelocity());

        telemetry.addData("currentLeft", leftcascade.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("currentRight", rightcascade.getCurrent(CurrentUnit.AMPS));

    }
}


