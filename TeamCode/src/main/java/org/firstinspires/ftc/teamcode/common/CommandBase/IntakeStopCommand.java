package org.firstinspires.ftc.teamcode.common.CommandBase;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;

public class IntakeStopCommand extends InstantCommand {
    public IntakeStopCommand() {
        super(
                (() -> Intake.stopIntaking())
        );
    }
}
