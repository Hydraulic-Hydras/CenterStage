package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake extends Contraption {
    private CRServo Wheels;
    private CRServo Zip;
    private CRServo Intake;
    private CRServo Outtake;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void initialize(HardwareMap hwMap) {
        Wheels = hwMap.get(CRServo.class,"Wheels");
        Zip = hwMap.get(CRServo.class, "Zip");
        Intake = hwMap.get(CRServo.class, "Intake");

        Outtake = hwMap.get(CRServo.class, "Outtake");

    }

    @Override
    public void loop(Gamepad gamepad1) {
        if (gamepad1.right_trigger > 0) {
            // Intake
            // zip at 80% so doesn't mess up
            Wheels.setPower(1);
            Zip.setPower(0.9);
            Intake.setPower(1);
        } else if (gamepad1.left_trigger > 0) {
            // Outtake
            Intake.setPower(-1);
            Wheels.setPower(-1);
            Zip.setPower(-0.9);
        } else {
            Wheels.setPower(0);
            Intake.setPower(0);
            Zip.setPower(0);
        }
    }

    public void outtakeLoop(Gamepad gamepad1) {
        if (gamepad1.circle) {
            // Outtake
            Outtake.setPower(-1);
        } else {
            Outtake.setPower(0);
        }
    }
}
