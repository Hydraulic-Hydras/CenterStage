package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Controller.PIDController;
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
    public static PIDController controller;

    /**
     * Sets PID gains to be used by the PIDF controller
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF kF
     */
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static int Target = 0;
    public static double Hold = 0.0001;
    public static double kG = Hold;

    public static int POS_REST = 0;
    public static int POS_LOW = 900;
    public static int POS_MID = 1500;
    public static int POS_HIGH = 2600;
    public Mitsumi(LinearOpMode opMode) { this.opMode = opMode; }
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


       // controller = new PIDController(kP, kI, kD, kF);
      //  controller.setPIDF(kP, kI, kD, kF);

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
            LeftCascade.setPower(gamepad2.right_trigger * 0.9);
            RightCascade.setPower(gamepad2.right_trigger * 0.9);
        } else if (gamepad2.left_trigger > 0 && !low_Limit.isPressed()) {
            // down
            LeftCascade.setPower(gamepad2.left_trigger * -0.82);
            RightCascade.setPower(gamepad2.left_trigger * -0.82);
        } else {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }

    }

    public void AutoMoveTo(int newTarget, double power) {

        if (high_Limit.isPressed()) {
            LeftCascade.setPower(0);
            RightCascade.setPower(0);
        }   else {
            RightCascade.setTargetPosition(newTarget);
            LeftCascade.setTargetPosition(newTarget);

            LeftCascade.setPower(power);
            RightCascade.setPower(power);

            LeftCascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightCascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }




    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Position: ", LeftCascade.getCurrentPosition());
        telemetry.addData("Right Position: ", RightCascade.getCurrentPosition());
        telemetry.addLine();

        telemetry.addData("Direction of Left: ", LeftCascade.getDirection());
        telemetry.addData("Direction of Right: ", RightCascade.getDirection());
        telemetry.addLine();

        telemetry.addData("Power Left: ", LeftCascade.getPower());
        telemetry.addData("Power Right: ", RightCascade.getPower());
        telemetry.addLine();

        telemetry.update();
    }
}
