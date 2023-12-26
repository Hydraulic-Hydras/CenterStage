package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;

@Autonomous
public class LiftTest extends LinearOpMode {

    private Mitsumi mitsumi = new Mitsumi(this);
    @Override
    public void runOpMode() {
        mitsumi.initialize(hardwareMap);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                mitsumi.AutoMoveTo(500, 0.86);
                mitsumi.telemetry(telemetry);
            }
        }
    }
}
