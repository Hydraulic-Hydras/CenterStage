package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Climber extends Contraption {

    public static DcMotor leftHook;
    public static DcMotor rightHook;

    public Climber(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    @Override
    public void initialize(HardwareMap hwMap) {
        leftHook = hwMap.get(DcMotor.class, "climber-L");
        leftHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHook.setDirection(DcMotorSimple.Direction.FORWARD);

        rightHook = hwMap.get(DcMotor.class, "climber-R");
        rightHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHook.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop(Gamepad gamepad) {

    }
}
