package org.firstinspires.ftc.teamcode.common.Hardware;

import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.LEDS;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
public class Robot extends Contraption {

    public Mitsumi mitsumi = new Mitsumi(opMode);
    public Intake intake = new Intake(opMode);
    public LEDS leds = new LEDS(opMode);
    public SampleMecanumDrive drive;

    public Robot(LinearOpMode opMode) { this.opMode = opMode; }
    @Override
    public void initialize(HardwareMap hwMap) {
        Globals.IS_AUTO = true;

        drive = new SampleMecanumDrive(hwMap);
        mitsumi.initialize(hwMap);
        mitsumi.autoInit();
        intake.initialize(hwMap);
        leds.initialize(hwMap);

    }
}
