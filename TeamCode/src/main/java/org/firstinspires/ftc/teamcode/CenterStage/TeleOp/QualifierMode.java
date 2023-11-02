package org.firstinspires.ftc.teamcode.CenterStage.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStage.Hardware.Camera;
import org.firstinspires.ftc.teamcode.CenterStage.Hardware.Cascades;
import org.firstinspires.ftc.teamcode.CenterStage.Hardware.Climbers;
import org.firstinspires.ftc.teamcode.CenterStage.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.CenterStage.Hardware.Intake;
import org.firstinspires.ftc.teamcode.CenterStage.Hardware.Launcher;

@TeleOp (name = "Qualifier TeleOp")
public class QualifierMode extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain(this);
    // Camera camera = new Camera(this, "Webcam");
    // Launcher launcher = new Launcher(this);
    // Cascades cascades = new Cascades(this);
    // Intake intake = new Intake(this);
    // Climbers climbers = new Climbers(this);


    @Override
    public void runOpMode() {
        driveTrain.init(hardwareMap);
        // climbers.init(hardwareMap);
        // intake.init(hardwareMap);
        // cascades.init(hardwareMap);
        // camera.init(hardwareMap);
        // launcher.init(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
               driveTrain.RobotCentric(gamepad1);
               //climbers.loop(gamepad1);
               // launcher.loop(gamepad2);
               // cascades.loop(gamepad2);
                // intake.loop(gamepad1);

                // climbers.telemetry(telemetry);
                // intake.telemetry(telemetry);
                // cascades.telemetry(telemetry);
                // launcher.telemetry(telemetry);
                // camera.telemetry(telemetry);
            }
        }

        // camera.stopStreaming();
    }
}
