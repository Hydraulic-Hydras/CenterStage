package org.firstinspires.ftc.teamcode.CenterStage.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Contraption {

    Servo CLAW;
    CRServo intake;

    // for Servos
    public static double OPEN_POS = 0;
    public static double CLOSE_POS = 1;

    // for CRServos
    public static double LEFT_POWER = -1;
    public static double RIGHT_POWER = 1;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        CLAW = hwMap.get(Servo.class, "9/11");
    }

    public void intake() {
        CLAW.setPosition(CLOSE_POS);

        intake.setPower(RIGHT_POWER);

    }

    public void outtake() {
        CLAW.setPosition(OPEN_POS);

        intake.setPower(LEFT_POWER);
    }

    @Override
    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.cross) {
            intake();
        } else if (gamepad1.circle) {
            outtake();
        }
    }
}
