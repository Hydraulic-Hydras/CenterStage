package org.firstinspires.ftc.teamcode.common.CommandBase;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;

@Config
public class IntakeReverseCommand extends InstantCommand {
    public IntakeReverseCommand() {
        super(
                (() -> Intake.reverseIntake())
        );
    }
}
