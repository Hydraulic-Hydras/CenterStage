package org.firstinspires.ftc.teamcode.common.Hardware.Contraptions;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ScoringFSM extends Contraption {

    public static double DELAY_PREP = 0.1;

    private Mitsumi slides = new Mitsumi(opMode);
    private Intake intake = new Intake(opMode);

    private ElapsedTime time;

    public ScoringFSM(LinearOpMode opMode) { this.opMode = opMode;}

    @Override
    public void initialize(HardwareMap hardwareMap) {
        slides.initialize(hardwareMap);
        intake.initialize(hardwareMap);

        time = new ElapsedTime();

    }

    public enum LiftState {
        REST,
        PREPARING,
        WAIT_TO_EXTEND,
        EXTENDING,
        WAIT_TO_RETRACT,
        SCORING,
        RETRACTING
    }

    public LiftState state = LiftState.REST;

    public enum LiftLevel {
        LOW,
        MEDIUM,
        HIGH
    }

    public LiftLevel level = LiftLevel.LOW;




}
