package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class RobotHardware {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DigitalChannel ALED_Green;
    private DigitalChannel ALED_Red;
    private DigitalChannel BLED_Green;
    private DigitalChannel BLED_Red;
    private DigitalChannel LED_GreenL;
    private DigitalChannel LED_RedL;
    private DigitalChannel LED_GreenR;
    private DigitalChannel LED_RedR;
    private DcMotor climberL;
    private DcMotor climberR;
    private DcMotor LeftCascade;
    private DcMotor RightCascade;
    private Servo rotateBucket;
    private Servo launcher_angle;
    private Servo claw;
    private Servo pixelRetainer;
    private Servo droneTrigger;
    private CRServo Wheels;
    private CRServo Zip;
    private CRServo Intake;
    private DistanceSensor distanceBackdrop;
    private TouchSensor low_Limit;
    private TouchSensor high_Limit;

    public double powerMultiplier;
    public double count;

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        LED_GreenL = hardwareMap.get(DigitalChannel.class, "LED_Green-L");
        LED_RedL = hardwareMap.get(DigitalChannel.class, "LED_Red-L");
        LED_GreenR = hardwareMap.get(DigitalChannel.class, "LED_Green-R");
        LED_RedR = hardwareMap.get(DigitalChannel.class, "LED_Red-R");
        ALED_Green = hardwareMap.get(DigitalChannel.class, "ALED_Green");
        ALED_Red = hardwareMap.get(DigitalChannel.class, "ALED_Red");
        BLED_Green = hardwareMap.get(DigitalChannel.class, "BLED_Green");
        BLED_Red = hardwareMap.get(DigitalChannel.class, "BLED_Red");

        climberL = hardwareMap.get(DcMotor.class, "climber-L");
        climberR = hardwareMap.get(DcMotor.class, "climber-R");
        LeftCascade = hardwareMap.get(DcMotor.class, "LeftCascade");
        RightCascade = hardwareMap.get(DcMotor.class, "RightCascade");
        rotateBucket = hardwareMap.get(Servo.class, "rotateBucket");
        launcher_angle = hardwareMap.get(Servo.class, "launcher_angle");
        claw = hardwareMap.get(Servo.class, "claw");
        pixelRetainer = hardwareMap.get(Servo.class, "pixelRetainer");
        droneTrigger = hardwareMap.get(Servo.class, "droneTrigger");
        distanceBackdrop = hardwareMap.get(DistanceSensor.class, "distanceBackdrop");
        low_Limit = hardwareMap.get(TouchSensor.class, "low_Limit");
        Wheels = hardwareMap.get(CRServo.class, "Wheels");
        Zip = hardwareMap.get(CRServo.class, "Zip");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        high_Limit = hardwareMap.get(TouchSensor.class, "high_Limit");

        // Put initialization blocks here.
        LED_GreenL.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RedL.setMode(DigitalChannel.Mode.OUTPUT);
        LED_GreenR.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RedR.setMode(DigitalChannel.Mode.OUTPUT);
        ALED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        ALED_Red.setMode(DigitalChannel.Mode.OUTPUT);
        BLED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        BLED_Red.setMode(DigitalChannel.Mode.OUTPUT);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        climberL.setDirection(DcMotor.Direction.FORWARD);
        climberR.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // intake position
        rotateBucket.setPosition(0.45);
        // horzontal angle
        launcher_angle.setPosition(0.6);
        claw.setPosition(0.4);
        // shooting angle
        launcher_angle.setPosition(0.28);
        powerMultiplier = 1;
        pixelRetainer.setPosition(0.1);
        ALED_Green.setState(false);
        ALED_Red.setState(true);
        BLED_Green.setState(false);
        BLED_Red.setState(true);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        // DRIVETRAIN
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;
        double leftFrontSpeed = vertical + horizontal + pivot;
        double leftRearSpeed = (vertical - horizontal) + pivot;
        double rightFrontSpeed = (vertical - horizontal) - pivot;
        double rightRearSpeed = (vertical + horizontal) - pivot;
        leftFront.setPower(leftFrontSpeed * powerMultiplier);
        leftRear.setPower(leftRearSpeed * powerMultiplier);
        rightFront.setPower(rightFrontSpeed * powerMultiplier);
        rightRear.setPower(rightRearSpeed * powerMultiplier);

        // Gear shift
        if (gamepad1.left_bumper) {
            powerMultiplier = 0.4;
        } else {
            powerMultiplier = 1;
        }

        // Bucket
        if (gamepad2.square) {
            // Reset
            rotateBucket.setPosition(0.4);
        } else if (gamepad2.cross && !low_Limit.isPressed()) {
            // Panel
            rotateBucket.setPosition(0.7);
        } else if (gamepad2.circle && !low_Limit.isPressed()) {
            // Drop
            rotateBucket.setPosition(0.84);
        }

        // Intake
        if (gamepad1.right_trigger > 0) {
            // Intake
            Wheels.setPower(1);
            Zip.setPower(1);
            Intake.setPower(1);
        } else if (gamepad1.left_trigger > 0) {
            // Outtake
            Intake.setPower(-1);
            Wheels.setPower(-1);
            Zip.setPower(-1);
        } else {
            Wheels.setPower(0);
            Intake.setPower(0);
            Zip.setPower(0);
        }

        // Slides
        if (gamepad2.left_trigger > 0 && !low_Limit.isPressed()) {
            // down
            LeftCascade.setPower(gamepad2.left_trigger * 0.6);
            RightCascade.setPower(gamepad2.left_trigger * 0.6);
        } else if (gamepad2.right_trigger > 0 && !high_Limit.isPressed()) {
            // up
            LeftCascade.setPower(gamepad2.right_trigger * -0.8);
            RightCascade.setPower(gamepad2.right_trigger * -0.8);
        } else {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }

        // Retainer
        if (gamepad2.left_bumper) {
            // close
            pixelRetainer.setPosition(0.05);
            ALED_Green.setState(true);
            ALED_Red.setState(false);
            BLED_Green.setState(true);
            BLED_Red.setState(false);
        } else if (gamepad2.y) {
            // open
            pixelRetainer.setPosition(0.15);
            ALED_Green.setState(false);
            ALED_Red.setState(true);
            BLED_Green.setState(false);
            BLED_Red.setState(true);
        }

        // launcher
        if (gamepad1.dpad_up) {
            // shooting angle
            launcher_angle.setPosition(0.28);
            launcher_angle.setPosition(0.34);
        } else if (gamepad1.dpad_down) {
            // horzontal angle
            launcher_angle.setPosition(0.5);
            launcher_angle.setPosition(0.6);
        }
        if (gamepad1.dpad_left) {
            // standby
            droneTrigger.setPosition(0);
        } else if (gamepad1.back) {
            // shoot
            droneTrigger.setPosition(0.9);
        }

        // climber
        if (gamepad1.y) {
            climberL.setPower(1);
            climberR.setPower(1);
        } else if (gamepad1.right_bumper) {
            climberL.setPower(-1);
            climberR.setPower(-1);
        } else {
            climberL.setPower(0);
            climberR.setPower(0);
        }

        if (launcher_angle.getPosition() < 0.4) {
            if (droneTrigger.getPosition() > 0.5) {
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
        }
        if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) >= 16 && Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 22) {
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

    }
}
