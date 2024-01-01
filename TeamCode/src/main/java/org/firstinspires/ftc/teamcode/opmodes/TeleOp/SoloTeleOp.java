package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ClassUtil;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.Util.LogFiles;

@TeleOp (name = "Solo TeleOp")
public class SoloTeleOp extends LinearOpMode {

    private Drivetrain drive = new Drivetrain(this);
    private Mitsumi slides = new Mitsumi(this);
    private Intake intake = new Intake(this);
    private Launcher drone = new Launcher(this);

    private LogFiles logFiles = new LogFiles(telemetry);

    @Override
    public void runOpMode() {
        drive.initialize(hardwareMap);
        slides.initialize(hardwareMap);
        intake.initialize(hardwareMap);
        drive.initialize(hardwareMap);

        logFiles.Telemetry(telemetry);

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                drive.RobotCentric(gamepad1);
                slides.loopSolo(gamepad1);
                intake.loop(gamepad1);
                intake.outtakeLoop(gamepad1);
                drone.loop(gamepad1);

                logFiles.Telemetry(telemetry);
            }
        }
    }

}
