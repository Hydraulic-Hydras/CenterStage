package org.firstinspires.ftc.teamcode.CenterStage.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Launcher extends Contraption {

    Servo launcher_angle;
    Servo launcher_trigger;
    public static double START_POS_ANGLE = 0.65;
    public static double START_POS_TRIGGER = 0.4;
    public static double HIGH_POS = 0.65;
    public static double MID_POS = 0.45;
    public static double LOW_POS = 0.25;
    public static double SHOOT = 0.75;
    public static double LOAD = 0.4;
    public Launcher(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {

        launcher_angle = hwMap.get(Servo.class, "launcher_angle");
        launcher_trigger = hwMap.get(Servo.class, "launcher_trigger");

        launcher_angle.setDirection(Servo.Direction.REVERSE);
        launcher_trigger.setDirection(Servo.Direction.FORWARD);

        launcher_angle.setPosition(START_POS_ANGLE);
        launcher_trigger.setPosition(START_POS_TRIGGER);
    }

    @Override
    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.triangle) {
            launcher_angle.setPosition(HIGH_POS);
        }   else if (gamepad1.square) {
            launcher_angle.setPosition(MID_POS);
        }   else if (gamepad1.cross) {
            launcher_angle.setPosition(LOW_POS);
        }

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            launcher_trigger.setPosition(SHOOT);
        }   else if (gamepad1.dpad_up) {
            launcher_trigger.setPosition(LOAD);
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Angle", launcher_angle.getPosition());
        telemetry.addData("Trigger", launcher_trigger.getPosition());
        telemetry.update();
    }
}
