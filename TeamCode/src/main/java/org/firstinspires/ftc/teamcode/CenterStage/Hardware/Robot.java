package org.firstinspires.ftc.teamcode.CenterStage.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Robot extends Contraption {

    DriveTrain driveTrain;
    Launcher launcher;

    public Robot(LinearOpMode OpMode) {
        this.opMode = OpMode;
    }
    public void init(HardwareMap hwMap) {
        driveTrain.init(hwMap);
        launcher.init(hwMap);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        driveTrain.loop(gamepad1);
        launcher.loop(gamepad2);
    }

    public void telemetry(Telemetry telemetry) {
        driveTrain.telemetry(telemetry);
        launcher.telemetry(telemetry);
    }
}
