package org.firstinspires.ftc.teamcode.OpModes.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;

@TeleOp (name = "Solo TeleOp" )
public class SoloTeleOp extends LinearOpMode {

    private DcMotorEx leftFront, rightRear, leftRear, rightFront;
    private DcMotorEx LeftCascade, RightCascade;
    public DcMotorEx leftHook, rightHook;
    public Servo rotateBucket, pixelRetainer, Dwayne, droneTrigger, launcher_angle;
    public CRServo Zip, intake, Wheels;
    public TouchSensor high_Limit, low_Limit;

    @Override
    public void runOpMode() {
        Globals.IS_AUTO = false;

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

        // CLIMBER
        leftHook = hardwareMap.get(DcMotorEx.class, "climber-L");
        leftHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHook.setDirection(DcMotorSimple.Direction.REVERSE);

        rightHook = hardwareMap.get(DcMotorEx.class, "climber-R");
        rightHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHook.setDirection(DcMotorSimple.Direction.REVERSE);

        // INTAKE
        Wheels = hardwareMap.get(CRServo.class, "Wheels");
        Zip = hardwareMap.get(CRServo.class, "Zip");
        intake = hardwareMap.get(CRServo.class, "Intake");
        pixelRetainer = hardwareMap.get(Servo.class, "pixelRetainer");
        Dwayne = hardwareMap.get(Servo.class, "claw");

        rotateBucket = hardwareMap.get(Servo.class, "rotateBucket");
        rotateBucket.setPosition(Intake.POS_REST);

        // DRONE SHOOTER
        launcher_angle = hardwareMap.get(Servo.class, "launcher_angle");
        droneTrigger = hardwareMap.get(Servo.class, "droneTrigger");

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

                leftFront.setPower(leftFrontSpeed);
                rightFront.setPower(rightFrontSpeed);
                leftRear.setPower(leftRearSpeed);
                rightRear.setPower(rightRearSpeed);

                // Slides
                if (gamepad1.right_bumper && !high_Limit.isPressed()) {
                    // up
                    LeftCascade.setPower(0.85);
                    RightCascade.setPower(0.85);
                } else if (gamepad1.left_bumper && !low_Limit.isPressed()) {
                    // down
                    LeftCascade.setPower(-0.5);
                    RightCascade.setPower(-0.5);
                } else {
                    LeftCascade.setPower(0);
                    RightCascade.setPower(0);
                }


                // Angle adjusting
                if (gamepad1.options) {
                    // shooting angle
                    launcher_angle.setPosition(0.34);
                } else if (gamepad1.share) {
                    // horizontal angle
                    launcher_angle.setPosition(0.6);
                }

                // Trigger controls
                if (gamepad1.dpad_left) {
                    // standby
                    droneTrigger.setPosition(0);
                } else if (gamepad1.dpad_right) {
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

                if (gamepad1.cross) {
                    // Reset
                    rotateBucket.setPosition(Intake.POS_REST);
                } else if (gamepad1.square && !low_Limit.isPressed()) {
                    // Panel
                    rotateBucket.setPosition(Intake.POS_PANEL);
                } else if (gamepad1.circle && !low_Limit.isPressed()) {
                    // Drop
                    rotateBucket.setPosition(Intake.POS_DUMP);
                }

                if (gamepad1.dpad_up) {
                    leftHook.setPower(1);
                    rightHook.setPower(1);
                } else if (gamepad1.dpad_down) {
                    leftHook.setPower(-1);
                    rightHook.setPower(-1);
                } else {
                    leftHook.setPower(0);
                    rightHook.setPower(0);
                }

            }
        }
    }
}
