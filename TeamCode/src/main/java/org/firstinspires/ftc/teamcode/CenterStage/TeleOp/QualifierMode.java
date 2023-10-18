package org.firstinspires.ftc.teamcode.CenterStage.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStage.Hardware.Camera;
import org.firstinspires.ftc.teamcode.CenterStage.Hardware.DriveTrain;

@TeleOp (name = "Qualifier TeleOp")
public class QualifierMode extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain(this);
    Camera camera = new Camera(this, "Webcam");
    // Launcher launcher = new Launcher(this);

    @Override
    public void runOpMode() {
        driveTrain.init(hardwareMap);
        camera.init(hardwareMap);
       // launcher.init(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
               driveTrain.RobotCentric(gamepad1);
               // launcher.loop(gamepad1);

                // launcher.telemetry(telemetry);
                camera.telemetry(telemetry);
               // driveTrain.FieldCentric(gamepad1);
            }
        }

        camera.stopStreaming();
    }
}
