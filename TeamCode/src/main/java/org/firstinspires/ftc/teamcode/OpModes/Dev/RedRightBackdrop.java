package org.firstinspires.ftc.teamcode.OpModes.Dev;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tuning.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.LEDS;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;

@Autonomous
public class RedRightBackdrop extends LinearOpMode {
    // Blue Prop (States).tflite
    // Red Prop (States).tflite
    private final Intake intake = new Intake(this);
    private final Mitsumi mitsumi = new Mitsumi(this);
    public SampleMecanumDrive drive;
    private final LEDS leds = new LEDS(this);
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = true;

        drive = new SampleMecanumDrive(hardwareMap);
        intake.initialize(hardwareMap);
        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        leds.initialize(hardwareMap);

        while (!isStarted()) {

        }

        Pose2d startPose = Globals.RedRight_StartPoseBK;
        drive.setPoseEstimate(startPose);

    }
}
