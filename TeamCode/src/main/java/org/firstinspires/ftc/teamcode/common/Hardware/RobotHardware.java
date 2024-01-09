package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    // Intake
    public Servo rotateBucket;
    public CRServo Zip;
    public CRServo Intake;
    public CRServo Wheels;

    // Drone
    public Servo launcher_angle;
    public Servo droneTrigger;

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

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        // SLIDES
        LeftCascade = hardwareMap.get(DcMotorEx.class, "LeftCascade");
        LeftCascade.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightCascade = hardwareMap.get(DcMotorEx.class, "RightCascade");
        RightCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightCascade.setDirection(DcMotorSimple.Direction.REVERSE);

        // INTAKE
        Wheels = hardwareMap.get(CRServo.class, "Wheels");
        Zip = hardwareMap.get(CRServo.class, "Zip");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        rotateBucket = hardwareMap.get(Servo.class, "rotateBucket");

        // DRONE SHOOTER
        launcher_angle = hardwareMap.get(Servo.class, "launcher_angle");
        droneTrigger = hardwareMap.get(Servo.class, "droneTrigger");

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
}
