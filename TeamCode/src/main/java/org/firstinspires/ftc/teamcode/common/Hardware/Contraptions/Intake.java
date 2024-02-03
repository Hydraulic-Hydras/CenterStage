package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Hardware.Globals;

@Config
public class Intake extends Contraption {

    public static CRServo Wheels;
    public static CRServo Zip;
    public static CRServo intake;
    public static Servo rotateBucket;
    public static Servo pixelRetainer;
    public static Servo DWAYNE;

    public static double POS_REST = 0.4;
    public static double POS_PANEL = 0.73;
    public static double POS_DUMP = 0.9;
    public static double POS_DOUBLE_DUMP = 1;

    public static boolean IS_INTAKING = false;
    public static boolean IS_REVERSED = false;

    public static double DWAYNE_CLOSE = 0.55;
    public static double DWAYNE_OVERLOAD_EXTEND = 0;

    public static double retainerOpen = 0.17;
    public static double retainerClose = 0.35;

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
        intake = hwMap.get(CRServo.class, "Intake");

        rotateBucket = hwMap.get(Servo.class, "rotateBucket");
        pixelRetainer = hwMap.get(Servo.class, "pixelRetainer");
        DWAYNE = hwMap.get(Servo.class, "claw");

        // Default position
        rotateBucket.setPosition(POS_REST);

        if (Globals.IS_AUTO) {
            DWAYNE.setPosition(DWAYNE_CLOSE);
        }   else {
            DWAYNE.setPosition(0.48);
        }
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            // Intake
            Wheels.setPower(1);
            Zip.setPower(1);
            intake.setPower(1);
        } else if (gamepad.left_trigger > 0) {
            // Outtake
            intake.setPower(-1);
            Wheels.setPower(-1);
            Zip.setPower(-1);
        } else {
            Wheels.setPower(0);
            intake.setPower(0);
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
            pixelRetainer.setPosition(retainerOpen);
        } else if (gamepad.y) {
            // grab
            pixelRetainer.setPosition(retainerClose);
        }
    }

    public static void startIntaking() {
        IS_INTAKING = true;
        IS_REVERSED = false;
        Wheels.setPower(1);
        Zip.setPower(1);
        intake.setPower(1);
    }

    public static void stopIntaking() {
        IS_INTAKING = false;
        IS_REVERSED = false;
        Wheels.setPower(0);
        intake.setPower(0);
        Zip.setPower(0);
    }

    public static void reverseIntake() {
        IS_REVERSED = true;
        IS_INTAKING = false;
        intake.setPower(-0.7);
        Wheels.setPower(-0.7);
        Zip.setPower(-0.7);
    }

    public static void fingerDown() {
        DWAYNE.setPosition(DWAYNE_OVERLOAD_EXTEND);
    }

    public static void fingerReset() {
        DWAYNE.setPosition(0.4);
    }

    public static void retainerOpen() {
        pixelRetainer.setPosition(retainerOpen);
    }

    public static void retainerClose() {
        pixelRetainer.setPosition(retainerClose);
    }

    public static State getState() {
        return outtakeState;
    }

}
