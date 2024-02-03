package org.firstinspires.ftc.teamcode.OpModes.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp (name = "Solo TeleOp" )
public class SoloTeleOp extends LinearOpMode {

    private DcMotorEx leftFront, rightRear, leftRear, rightFront;
    private DcMotorEx LeftCascade, RightCascade;
    public Servo rotateBucket, pixelRetainer, Dwayne, droneTrigger, launcher_angle;
    public CRServo Zip, intake, Wheels;
    public TouchSensor high_Limit, low_Limit;
    public double powerMultiplier;

    @Override
    public void runOpMode() {
        // DRIVETRAIN
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        intake = hardwareMap.get(CRServo.class, "Intake");
        pixelRetainer = hardwareMap.get(Servo.class, "pixelRetainer");
        Dwayne = hardwareMap.get(Servo.class, "claw");

        rotateBucket = hardwareMap.get(Servo.class, "rotateBucket");
        rotateBucket.setPosition(0.2);

        // DRONE SHOOTER
        launcher_angle = hardwareMap.get(Servo.class, "launcher_angle");
        droneTrigger = hardwareMap.get(Servo.class, "droneTrigger");

        launcher_angle.setPosition(0.28);

        // Sensors
        high_Limit = hardwareMap.get(TouchSensor.class, "high_Limit");
        low_Limit = hardwareMap.get(TouchSensor.class, "low_Limit");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Mecanum Wheel Drivetrain control
                double vertical = -gamepad1.left_stick_y;
                double horizontal = gamepad1.left_stick_x;
                double pivot = gamepad1.right_stick_x;

                double rightFrontSpeed = ((vertical - horizontal) - pivot);
                double rightRearSpeed = ((vertical + horizontal) - pivot);
                double leftFrontSpeed = (vertical + horizontal + pivot);
                double leftRearSpeed = ((vertical - horizontal) + pivot);

                leftFront.setPower(leftFrontSpeed * powerMultiplier);
                rightFront.setPower(rightFrontSpeed * powerMultiplier);
                leftRear.setPower(leftRearSpeed * powerMultiplier);
                rightRear.setPower(rightRearSpeed * powerMultiplier);

                if (gamepad1.left_bumper) {
                    powerMultiplier = 0.4;
                } else {
                    powerMultiplier = 1;
                }

                // Slides
                if (gamepad2.right_trigger > 0 && !high_Limit.isPressed()) {
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

                // Angle adjusting
                if (gamepad1.dpad_up) {
                    // shooting angle
                    launcher_angle.setPosition(0.52);
                } else if (gamepad1.dpad_down) {
                    // horizontal angle
                    launcher_angle.setPosition(0.35);
                }

                // Trigger controls
                if (gamepad1.dpad_left) {
                    // standby
                    droneTrigger.setPosition(0);
                } else if (gamepad1.share) {
                    // shoot
                    droneTrigger.setPosition(0.9);
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

                // Bucket controls
                if (gamepad2.a) {
                    // bucket Position
                    rotateBucket.setPosition(0.2);
                } else if (gamepad2.b && !low_Limit.isPressed()) {
                    if (rotateBucket.getPosition() == 1 || rotateBucket.getPosition() == 0) {
                        // Parallel
                        rotateBucket.setPosition(0.5);
                    } else if (rotateBucket.getPosition() == 0.5) {
                        // Drop
                        rotateBucket.setPosition(1);
                    }
                }

                // Pixel retainer controls
                if (gamepad2.x) {
                    // open
                    pixelRetainer.setPosition(0.46);
                } else if (gamepad2.y) {
                    // grab
                    pixelRetainer.setPosition(0.42);
                }

            }
        }
    }
}
