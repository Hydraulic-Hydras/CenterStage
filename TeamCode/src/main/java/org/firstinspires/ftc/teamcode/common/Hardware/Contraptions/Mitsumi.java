package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Mitsumi extends Contraption {

    public static TouchSensor high_Limit;
    public static TouchSensor low_Limit;
    public static DcMotor LeftCascade;
    public static DcMotor RightCascade;

    public Mitsumi(LinearOpMode opMode) { this.opMode = opMode; }
    public enum liftState {
        REST,
        RAMPING,
        EXTENDED
    }

    liftState liftstate = liftState.REST;
    @Override
    public void initialize(HardwareMap hwMap) {

        LeftCascade = hwMap.get(DcMotor.class, "LeftCascade");
        RightCascade = hwMap.get(DcMotor.class, "RightCascade");

        high_Limit = hwMap.get(TouchSensor.class, "high_Limit");
        low_Limit = hwMap.get(TouchSensor.class, "low_Limit");

        LeftCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightCascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftCascade.setDirection(DcMotorSimple.Direction.REVERSE);
        RightCascade.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void autoInit() {
        RightCascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftCascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftCascade.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightCascade.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void loop(Gamepad gamepad2) {
        // Put loop blocks here.
        if (gamepad2.right_trigger > 0 && !high_Limit.isPressed() ) {
            // up
            LeftCascade.setPower(gamepad2.right_trigger * 0.8);
            RightCascade.setPower(gamepad2.right_trigger * 0.8);
        } else if (gamepad2.left_trigger > 0 && !low_Limit.isPressed()) {
            // down
            LeftCascade.setPower(gamepad2.left_trigger * -0.6);
            RightCascade.setPower(gamepad2.left_trigger * -0.6);
        } else {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }

        if (low_Limit.isPressed() && !high_Limit.isPressed()) {
            liftstate = liftState.REST;
        }   else if (high_Limit.isPressed() && !low_Limit.isPressed()) {
            liftstate = liftState.EXTENDED;
        }   else {
            liftstate = liftState.RAMPING;
        }

    }

    public void autoMoveTo(int pos, double power) {
        LeftCascade.setTargetPosition(pos);
        RightCascade.setTargetPosition(pos);

        LeftCascade.setPower(power);
        RightCascade.setPower(power);

        LeftCascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightCascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void extendHigh() {

        if (!high_Limit.isPressed()) {
            LeftCascade.setPower(0.7);
            RightCascade.setPower(0.7);
        }   else {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }
    }

    public void extendLow() {

        if (!low_Limit.isPressed()) {
            LeftCascade.setPower(-0.65);
            RightCascade.setPower(-0.65);
        }   else {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Position: ", LeftCascade.getCurrentPosition());
        telemetry.addData("Right Position: ", RightCascade.getCurrentPosition());
        telemetry.addLine();

        telemetry.addData("Direction of Left: ", LeftCascade.getDirection());
        telemetry.addData("Direction of Right: ", RightCascade.getDirection());
        telemetry.addLine();

        telemetry.update();
    }
}
