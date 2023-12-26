package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;

@TeleOp ( name = "JAVA-LiftTest")
public class LiftTest extends LinearOpMode {

    private Mitsumi mitsumi = new Mitsumi(this);
    @Override
    public void runOpMode() {
        mitsumi.initialize(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (gamepad1.y) {
                    mitsumi.extendHigh();
                }   else if (gamepad1.x) {
                    mitsumi.extendMid();
                }   else if (gamepad1.a) {
                    mitsumi.extendLow();
                }   else if (gamepad1.b) {
                    mitsumi.setPower(0);
                }

                mitsumi.telemetry(telemetry);
            }
        }
    }
}
