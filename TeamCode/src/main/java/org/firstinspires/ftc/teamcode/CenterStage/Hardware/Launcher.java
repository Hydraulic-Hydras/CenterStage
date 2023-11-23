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

    public static double START_POS_ANGLE = 1;
    public static double START_POS_TRIGGER = 0.5;

    public static double HIGH_POS = 0.89;
    public static double MID_POS = 0.6;
    public static double LOW_POS = 0.35;

    public static double SHOOT = 0.2;
    public static double LOAD = 0.5;
    public Launcher(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize(HardwareMap hwMap) {

        launcher_angle = hwMap.get(Servo.class, "launcher_angle");
        launcher_trigger = hwMap.get(Servo.class, "launcher_trigger");

        launcher_angle.setDirection(Servo.Direction.REVERSE);
        launcher_trigger.setDirection(Servo.Direction.REVERSE);

        launcher_angle.setPosition(START_POS_ANGLE);
        launcher_trigger.setPosition(START_POS_TRIGGER);
    }

    public void loop(Gamepad gamepad2) {

        if (gamepad2.dpad_up) {
            // Triangle or Y button is High
            launcher_angle.setPosition(HIGH_POS);
        }   else if (gamepad2.dpad_right) {
            // Circle or B is Mid
            launcher_angle.setPosition(MID_POS);
        }   else if (gamepad2.dpad_down) {
            // Cross or A is low
            launcher_angle.setPosition(LOW_POS);
        }

        // both bumpers for shoot
        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            launcher_trigger.setPosition(SHOOT);
        }   else if (gamepad2.b) {
            // dpad down for loading
            launcher_trigger.setPosition(LOAD);
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Angle", launcher_angle.getPosition());
        telemetry.addData("Trigger", launcher_trigger.getPosition());
        telemetry.update();
    }
}
