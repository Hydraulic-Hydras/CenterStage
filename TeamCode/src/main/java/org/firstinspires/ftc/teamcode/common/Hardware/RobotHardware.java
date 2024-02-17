package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class RobotHardware {

    // DriveTrain
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;

    // Slides
    public DcMotorEx LeftCascade;
    public DcMotorEx RightCascade;

    // Climber
    public DcMotorEx leftHook;
    public DcMotorEx rightHook;

    // Intake
    public Servo rotateBucket;
    public CRServo Zip;
    public CRServo intake;
    public CRServo Wheels;
    public Servo pixelRetainer;

    // Drone
    public Servo launcher_angle;
    public Servo droneTrigger;
    public Servo Dwayne;

    // Imu
    public IMU imu;

    // Sensors
    public TouchSensor high_Limit;
    public TouchSensor low_Limit;
    public DistanceSensor distanceBackdrop;

    // LEDS & Digital Channels
    public DigitalChannel LED_GreenL;
    public DigitalChannel LED_RedL;
    public DigitalChannel LED_GreenR;
    public DigitalChannel LED_RedR;

    public DigitalChannel ALED_Green;
    public DigitalChannel ALED_Red;
    public DigitalChannel BLED_Green;
    public DigitalChannel BLED_Red;

    public static int count = 0;

    /**
     * Variables for Drone
     */
    public static double SHOOT_POS = 0.52;
    public static double HORIZONTAL_POS = 0.35;

    public static double INIT_POS = 0.28;

    public static double SHOOT = 0.9;
    public static double LOAD = 0;

    /**
     * Variables for Bucket
     */
    public static double POS_REST = 0.2;
    public static double POS_PANEL = 0.5;
    public static double POS_DUMP = 1;

    public void init(HardwareMap hardwareMap) {

        // DRIVETRAIN
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Globals.IS_AUTO) {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ));
            imu.initialize(parameters);
        }
        // SLIDES
        LeftCascade = hardwareMap.get(DcMotorEx.class, "LeftCascade");
        LeftCascade.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightCascade = hardwareMap.get(DcMotorEx.class, "RightCascade");
        RightCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightCascade.setDirection(DcMotorSimple.Direction.REVERSE);

        // CLIMBER
        leftHook = hardwareMap.get(DcMotorEx.class, "climber-L");
        leftHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHook.setDirection(DcMotorSimple.Direction.FORWARD);

        rightHook = hardwareMap.get(DcMotorEx.class, "climber-R");
        rightHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHook.setDirection(DcMotorSimple.Direction.FORWARD);

        // INTAKE
        Wheels = hardwareMap.get(CRServo.class, "Wheels");
        Zip = hardwareMap.get(CRServo.class, "Zip");
        intake = hardwareMap.get(CRServo.class, "Intake");
        rotateBucket = hardwareMap.get(Servo.class, "rotateBucket");
        pixelRetainer = hardwareMap.get(Servo.class, "pixelRetainer");
        Dwayne = hardwareMap.get(Servo.class, "claw");

        rotateBucket.setPosition(POS_REST);

        // DRONE SHOOTER
        launcher_angle = hardwareMap.get(Servo.class, "launcher_angle");
        droneTrigger = hardwareMap.get(Servo.class, "droneTrigger");

        launcher_angle.setPosition(SHOOT_POS);
        launcher_angle.setPosition(INIT_POS);

        // Sensors
        high_Limit = hardwareMap.get(TouchSensor.class, "high_Limit");
        low_Limit = hardwareMap.get(TouchSensor.class, "low_Limit");

        distanceBackdrop = hardwareMap.get(DistanceSensor.class, "distanceBackdrop");

        // LEDS
        LED_GreenL = hardwareMap.get(DigitalChannel.class, "LED_Green-L");
        LED_GreenL.setMode(DigitalChannel.Mode.OUTPUT);

        LED_RedL = hardwareMap.get(DigitalChannel.class, "LED_Red-L");
        LED_RedL.setMode(DigitalChannel.Mode.OUTPUT);

        LED_GreenR = hardwareMap.get(DigitalChannel.class, "LED_Green-R");
        LED_GreenR.setMode(DigitalChannel.Mode.OUTPUT);

        LED_RedR = hardwareMap.get(DigitalChannel.class, "LED_Red-R");
        LED_RedR.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Green = hardwareMap.get(DigitalChannel.class, "ALED_Green");
        ALED_Green.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Red = hardwareMap.get(DigitalChannel.class, "ALED_Red");
        ALED_Red.setMode(DigitalChannel.Mode.OUTPUT);

        BLED_Green = hardwareMap.get(DigitalChannel.class, "BLED_Green");
        BLED_Green.setMode(DigitalChannel.Mode.OUTPUT);

        BLED_Red = hardwareMap.get(DigitalChannel.class, "BLED_Red");
        BLED_Red.setMode(DigitalChannel.Mode.OUTPUT);

    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        // Drivetrain control (ROBOT CENTRIC)
        double powerMultiplier;
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

        // Slides
        if (gamepad2.right_trigger > 0 && !high_Limit.isPressed() ) {
            // up
            LeftCascade.setPower(gamepad2.right_trigger * 0.8);
            RightCascade.setPower(gamepad2.right_trigger * 0.8);
        } else if (gamepad2.left_trigger > 0 && !low_Limit.isPressed()) {
            // down
            LeftCascade.setPower(gamepad2.left_trigger * -0.6);
            RightCascade.setPower(gamepad2.left_trigger * -0.6);
        } else {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }

        // Intake controls
        if (gamepad1.right_trigger > 0) {
            // Intake
            Wheels.setPower(1);
            Zip.setPower(1);
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0) {
            // Outtake
            intake.setPower(-1);
            Wheels.setPower(-1);
            Zip.setPower(-1);
        } else {
            Wheels.setPower(0);
            intake.setPower(0);
            Zip.setPower(0);
        }

        // LED INPUTS
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


        if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) >= 16 && Double.parseDouble(
                JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 22
                && rotateBucket.getPosition() != 0.2) {
            ALED_Green.setState(true);
            ALED_Red.setState(false);
            BLED_Green.setState(true);
            BLED_Red.setState(false);
        } else if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 40) {
            ALED_Green.setState(false);
            ALED_Red.setState(true);
            BLED_Green.setState(false);
            BLED_Red.setState(true);
        } else if (launcher_angle.getPosition() < 0.4) {
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
        } else {
            ALED_Green.setState(true);
            ALED_Red.setState(true);
            BLED_Green.setState(true);
            BLED_Red.setState(true);
        }
    }
}
