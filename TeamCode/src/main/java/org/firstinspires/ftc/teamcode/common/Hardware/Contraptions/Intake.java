package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Contraption {

    public static CRServo Wheels;
    public static CRServo Zip;
    public static CRServo Intake;
    public static Servo rotateBucket;
    public static Servo pixelRetainer;

    public static double POS_REST = 0.2;
    public static double POS_PANEL = 0.5;
    public static double POS_DUMP = 1;

    public static boolean IS_INTAKING = false;
    public static boolean IS_REVERSED = false;

    public enum State {
        REST,
        PANEL,
        DUMP,
    }

    public static State outtakeState = State.REST;
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    @Override
    public void initialize(HardwareMap hwMap) {
        Wheels = hwMap.get(CRServo.class, "Wheels");
        Zip = hwMap.get(CRServo.class, "Zip");
        Intake = hwMap.get(CRServo.class, "Intake");

        rotateBucket = hwMap.get(Servo.class, "rotateBucket");
        pixelRetainer = hwMap.get(Servo.class, "pixelRetainer");


        // Default position
        rotateBucket.setPosition(POS_REST);

    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            // Intake
            Wheels.setPower(1);
            Zip.setPower(1);
            Intake.setPower(1);
        } else if (gamepad.left_trigger > 0) {
            // Outtake
            Intake.setPower(-1);
            Wheels.setPower(-1);
            Zip.setPower(-1);
        } else {
            Wheels.setPower(0);
            Intake.setPower(0);
            Zip.setPower(0);
        }

    }

    public void outtakeLoop(Gamepad gamepad) {

        if (gamepad.a) {
            // Intake Position
            rotateBucket.setPosition(POS_REST);
            outtakeState = State.REST;
        } else if (gamepad.b && !Mitsumi.low_Limit.isPressed()) {
            if (rotateBucket.getPosition() == 1 || rotateBucket.getPosition() == 0) {
                // Parallel
                rotateBucket.setPosition(POS_PANEL);
                outtakeState = State.PANEL;
            } else if (rotateBucket.getPosition() == POS_PANEL) {
                // Drop
                rotateBucket.setPosition(POS_DUMP);
                outtakeState = State.DUMP;
            }
        }

        if (gamepad.x) {
            // open
            pixelRetainer.setPosition(0.46);
        } else if (gamepad.y) {
            // grab
            pixelRetainer.setPosition(0.42);
        }
    }

    public static void startIntaking() {
        IS_INTAKING = true;
        IS_REVERSED = false;
        Wheels.setPower(1);
        Zip.setPower(1);
        Intake.setPower(1);
    }

    public static void stopIntaking() {
        IS_INTAKING = false;
        IS_REVERSED = false;
        Wheels.setPower(0);
        Intake.setPower(0);
        Zip.setPower(0);
    }

    public static void reverseIntake() {
        IS_REVERSED = true;
        IS_INTAKING = false;
        Intake.setPower(-0.7);
        Wheels.setPower(-0.7);
        Zip.setPower(-0.7);
    }

    public static State getState() {
        return outtakeState;
    }

}
