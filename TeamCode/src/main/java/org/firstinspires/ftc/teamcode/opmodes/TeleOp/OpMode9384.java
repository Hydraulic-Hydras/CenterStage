package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.Drivetrain;
@TeleOp (name = "Qual #2 TeleOp")
public class OpMode9384 extends LinearOpMode {

    private Intake intake = new Intake(this);
    private Drivetrain drive = new Drivetrain(this);
    private Mitsumi slides = new Mitsumi(this);
   // private Launcher launcher = new Launcher(this);

    @Override
    public void runOpMode() {
        intake.initialize(hardwareMap);
        drive.initialize(hardwareMap);
        slides.initialize(hardwareMap);
       // launcher.initialize(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive.RobotCentric(gamepad1);
                slides.loop(gamepad2);

                intake.loop(gamepad1);
                intake.outtakeLoop(gamepad2);
                //  launcher.loop(gamepad1);

                slides.telemetry(telemetry);
            }
        }
    }
}
