package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Mitsumi extends Contraption {

    private TouchSensor high_limit;
    private TouchSensor low_limit;
    private DcMotor LeftCascade;
    private DcMotor RightCascade;

    public Mitsumi(LinearOpMode opMode) { this.opMode = opMode; }
    @Override
    public void initialize(HardwareMap hwMap) {

        LeftCascade = hwMap.get(DcMotor.class, "LeftCascade");
        RightCascade = hwMap.get(DcMotor.class, "RightCascade");

        high_limit = hwMap.get(TouchSensor.class, "high_Limit");
        low_limit = hwMap.get(TouchSensor.class, "low_Limit");
    }

    public void loop(Gamepad gamepad2) {
        // Put loop blocks here.
        if (gamepad2.right_trigger > 0 && !high_limit.isPressed()) {
            // up
            LeftCascade.setPower(gamepad2.right_trigger * -0.86);
            RightCascade.setPower(gamepad2.right_trigger * -0.86);
        } else if (gamepad2.left_trigger > 0 && !low_limit.isPressed()) {
            // down
            LeftCascade.setPower(gamepad2.left_trigger * 0.8);
            RightCascade.setPower(gamepad2.left_trigger * 0.8);
        } else {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }
    }


    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Pos: ", LeftCascade.getCurrentPosition());
        telemetry.addData("Right Pos: ", RightCascade.getCurrentPosition());
        telemetry.update();
    }
}
