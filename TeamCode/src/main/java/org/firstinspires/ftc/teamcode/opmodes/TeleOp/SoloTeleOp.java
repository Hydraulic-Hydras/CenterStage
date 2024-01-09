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
        mitsumi.initialize(hardwareMap);
        launcher.initialize(hardwareMap);

        files.Telemetry(telemetry);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                drivetrain.RobotCentric(gamepad1);
                intake.loop(gamepad1);

                // Slides loop
                if (gamepad1.right_bumper && !Mitsumi.high_Limit.isPressed()) {
                    // up
                    Mitsumi.LeftCascade.setPower(0.7);
                    Mitsumi.RightCascade.setPower(0.7);
                }   else if (gamepad1.left_bumper && !Mitsumi.low_Limit.isPressed()) {
                    // down
                    Mitsumi.RightCascade.setPower(-0.62);
                    Mitsumi.LeftCascade.setPower(-0.62);
                }   else {
                    Mitsumi.LeftCascade.setPower(0);
                    Mitsumi.RightCascade.setPower(0);
                }

                // Outtake loop
                if (gamepad1.x) {
                    // Intake Position
                    Intake.rotateBucket.setPosition(Intake.POS_REST);
                    Intake.outtakeState = Intake.State.REST;
                } else if (gamepad1.y && !Mitsumi.low_Limit.isPressed()) {
                    if (Intake.rotateBucket.getPosition() == 1 || Intake.rotateBucket.getPosition() == 0) {
                        // Parallel
                        Intake.rotateBucket.setPosition(Intake.POS_PANEL);
                        Intake.outtakeState = Intake.State.PANEL;
                    } else if (Intake.rotateBucket.getPosition() == Intake.POS_PANEL) {
                        // Drop
                        Intake.rotateBucket.setPosition(Intake.POS_DUMP);
                        Intake.outtakeState = Intake.State.DUMP;
                    }
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