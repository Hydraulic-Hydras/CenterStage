package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Controller.PIDController;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Contraption {

    private CRServo Wheels;
    private CRServo Zip;
    private CRServo Intake;
    private Servo rotateBucket;

    public static double POS_REST = 0;
    public static double POS_PANEL = 0.5;
    public static double POS_DUMP = 1;
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void initialize(HardwareMap hwMap) {
        Wheels = hwMap.get(CRServo.class, "Wheels");
        Zip = hwMap.get(CRServo.class, "Zip");
        Intake = hwMap.get(CRServo.class, "Intake");

        rotateBucket = hwMap.get(Servo.class, "rotateBucket");

        // Default position
        rotateBucket.setPosition(0);

    }

    @Override
    public void loop(Gamepad gamepad1) {
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
    }

    public void outtakeLoop(Gamepad gamepad2) {

        if (gamepad2.x) {
            // reset
            rotateBucket.setPosition(POS_REST);
        } else if (gamepad2.a) {
            // dump
            rotateBucket.setPosition(POS_DUMP);
        } else if (gamepad2.b   ) {
            // panel
            rotateBucket.setPosition(POS_PANEL);
        }
    }

}
