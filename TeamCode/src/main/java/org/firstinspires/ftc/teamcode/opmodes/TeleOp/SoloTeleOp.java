package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.Util.InfoFiles;

@TeleOp (name = "Solo TeleOp")
public class SoloTeleOp extends LinearOpMode {

    private final Drivetrain drive = new Drivetrain(this);
    private final Mitsumi slides = new Mitsumi(this);
    private final Intake intake = new Intake(this);
    private final Launcher drone = new Launcher(this);

    private final InfoFiles infoFiles = new InfoFiles(telemetry);

    @Override
    public void runOpMode() {
        drive.initialize(hardwareMap);
        slides.initialize(hardwareMap);
        intake.initialize(hardwareMap);
        drive.initialize(hardwareMap);

        infoFiles.Telemetry(telemetry);

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                drive.RobotCentric(gamepad1);
                slides.loopSolo(gamepad1);
                intake.loop(gamepad1);
                intake.outtakeLoop(gamepad1);
                drone.loop(gamepad1);

                infoFiles.Telemetry(telemetry);
            }
        }
    }

}
