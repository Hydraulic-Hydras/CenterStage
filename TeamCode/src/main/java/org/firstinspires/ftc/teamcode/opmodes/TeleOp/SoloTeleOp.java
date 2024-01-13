package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.Util.TeleOpInfo;

@TeleOp (name = "SoloTeleOp", group = "TeleOp")
public class SoloTeleOp extends LinearOpMode {

    private final Drivetrain drivetrain = new Drivetrain(this);
    private final Intake intake = new Intake(this);
    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Launcher launcher = new Launcher(this);

    private final TeleOpInfo files = new TeleOpInfo(telemetry);

    @Override
    public void runOpMode() {
        intake.initialize(hardwareMap);
        drivetrain.initialize(hardwareMap);
        drivetrain.SensorInit(hardwareMap);
        mitsumi.initialize(hardwareMap);
        launcher.initialize(hardwareMap);

        files.Telemetry(telemetry);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                drivetrain.RobotCentric(gamepad1);

                // Slides loop
                if (gamepad1.right_trigger > 0 && !Mitsumi.high_Limit.isPressed()) {
                    // up
                    Mitsumi.LeftCascade.setPower(gamepad1.right_trigger * 0.7);
                    Mitsumi.RightCascade.setPower(gamepad1.right_trigger * 0.7);
                }   else if (gamepad1.left_trigger > 0 && !Mitsumi.low_Limit.isPressed()) {
                    // down
                    Mitsumi.RightCascade.setPower(gamepad1.left_trigger * -0.62);
                    Mitsumi.LeftCascade.setPower(gamepad1.left_trigger  * -0.62);
                }   else {
                    Mitsumi.LeftCascade.setPower(0);
                    Mitsumi.RightCascade.setPower(0);
                }

                // Intake loop
                if (gamepad1.x) {
                    Intake.Intake.setPower(1);
                    Intake.Zip.setPower(1);
                    Intake.Wheels.setPower(1);
                }   else if (gamepad1.y) {
                    Intake.Intake.setPower(-1);
                    Intake.Zip.setPower(-1);
                    Intake.Wheels.setPower(-1);
                }   else {
                    Intake.Intake.setPower(0);
                    Intake.Zip.setPower(0);
                    Intake.Wheels.setPower(0);
                }

                // Launcher loop
                if (gamepad1.dpad_up) {
                    // shooting angle
                    Launcher.launcher_angle.setPosition(Launcher.SHOOT_POS);
                    Launcher.droneAngle = Launcher.LauncherAngle.READY;
                } else if (gamepad1.dpad_down) {
                    // horizontal angle
                    Launcher.launcher_angle.setPosition(Launcher.HORIZONTAL_POS);
                    Launcher.droneAngle = Launcher.LauncherAngle.RESET;
                }

                if (gamepad1.dpad_left) {
                    // standby
                    Launcher.droneTrigger.setPosition(Launcher.LOAD);
                    Launcher.droneState = Launcher.LauncherState.LOADED;
                } else if (gamepad1.dpad_right) {
                    // shoot
                    Launcher.droneTrigger.setPosition(Launcher.SHOOT);
                    Launcher.droneState = Launcher.LauncherState.hasShot;
                }


                files.Telemetry(telemetry);

            }
        }
    }
}