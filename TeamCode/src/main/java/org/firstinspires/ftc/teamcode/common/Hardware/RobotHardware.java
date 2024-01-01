package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tuning.DriveConstants;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;
import org.firstinspires.ftc.teamcode.common.Util.HSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// TODO: finalize this file
@Config
public class RobotHardware {

    // Drivetrain
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;

    // Contraptions
    public DcMotorEx LeftCascade;
    public DcMotorEx RightCascade;

    public Servo Bucket;
    public Servo Finger;
    public CRServo Wheels;
    public CRServo Intake;
    public CRServo Zip;

    public Servo Launcher_Angle;
    public Servo Launcher_Trigger;

    // public DcMotorEx LeftHook;
    // public DcMotorEx RightHook;

    // Imu
    public IMU imu;

    /**
     * HardwareMap storage.
     */
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Sensors
    public TouchSensor high_Limit;
    public TouchSensor low_Limit;

    public DistanceSensor distanceBackdrop;

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
     * PID Values
     * @param kP Proportional term
     * @param kI Integral term
     * @param kD Derivative term
     */
    public static double kP = 0.26;
    public static double kI = 0;
    public static double kD = 0;

    /**
     * Voltage timer and voltage value.
     */
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    public boolean enabled;

    public List<LynxModule> modules;
    private ArrayList<HSubsystem> subsystems;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Values for all Contraptions
     */
    public static double INIT_POS = 0.28;
    public static double LOAD = 0;

    public static double POS_REST = 0;

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        this.subsystems = new ArrayList<>();
        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // DRIVETRAIN
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // INTAKE AND OUTTAKE
        Wheels = hardwareMap.get(CRServo.class, "Wheels");
        Zip = hardwareMap.get(CRServo.class, "Zip");
        Intake = hardwareMap.get(CRServo.class, "Intake");

        Bucket = hardwareMap.get(Servo.class, "rotateBucket");
        Finger = hardwareMap.get(Servo.class, "Finger");

        Bucket.setPosition(POS_REST);

        // DRONE
        Launcher_Angle = hardwareMap.get(Servo.class, "launcher_angle");
        Launcher_Trigger = hardwareMap.get(Servo.class, "launcher_trigger");

        Launcher_Angle.setPosition(INIT_POS);
        Launcher_Trigger.setPosition(LOAD);

        // SLIDES
        LeftCascade = hardwareMap.get(DcMotorEx.class, "LeftCascade");
        LeftCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftCascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftCascade.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftCascade.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, 0));

        RightCascade = hardwareMap.get(DcMotorEx.class, "RightCascade");
        RightCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightCascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightCascade.setDirection(DcMotorSimple.Direction.REVERSE);
        RightCascade.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, 0));

        // SENSORS
        high_Limit = hardwareMap.get(TouchSensor.class, "high_Limit");
        low_Limit = hardwareMap.get(TouchSensor.class, "low_Limit");

        LED_GreenL = hardwareMap.get(DigitalChannel.class, "LED_Green-L");
        LED_RedL = hardwareMap.get(DigitalChannel.class, "LED_Red-L");
        LED_GreenR = hardwareMap.get(DigitalChannel.class, "LED_Green-R");
        LED_RedR = hardwareMap.get(DigitalChannel.class, "LED_Red-R");

        distanceBackdrop = hardwareMap.get(DistanceSensor.class, "distanceBackdrop");

        LED_GreenL.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RedL.setMode(DigitalChannel.Mode.OUTPUT);
        LED_GreenR.setMode(DigitalChannel.Mode.OUTPUT);
        LED_RedR.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Green = hardwareMap.get(DigitalChannel.class, "ALED_Green");
        ALED_Red = hardwareMap.get(DigitalChannel.class, "ALED_Red");
        BLED_Green = hardwareMap.get(DigitalChannel.class, "BLED_Green");
        BLED_Red = hardwareMap.get(DigitalChannel.class, "BLED_Red");

        ALED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        ALED_Red.setMode(DigitalChannel.Mode.OUTPUT);
        BLED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        BLED_Red.setMode(DigitalChannel.Mode.OUTPUT);


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
                && org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake.rotateBucket.getPosition() != 0.2) {
            ALED_Green.setState(true);
            ALED_Red.setState(false);
            BLED_Green.setState(true);
            BLED_Red.setState(false);
        } else if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 40) {
            ALED_Green.setState(false);
            ALED_Red.setState(true);
            BLED_Green.setState(false);
            BLED_Red.setState(true);
        } else if (Launcher.launcher_angle.getPosition() < 0.4) {
            if (Launcher.droneTrigger.getPosition() > 0.5) {
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

    public void read() {
//        imuAngle = imu.getAngularOrientation().firstAngle;
        for (HSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (HSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void periodic() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

        for (HSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void reset() {
        for (HSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    public double getVoltage() {
        return voltage;
    }

    public void addSubsystem(HSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public void log(String data) {
        telemetry.addLine(data);
    }

    public void log(String data, Object input) {
        telemetry.addData(data, input.toString());
    }


}
