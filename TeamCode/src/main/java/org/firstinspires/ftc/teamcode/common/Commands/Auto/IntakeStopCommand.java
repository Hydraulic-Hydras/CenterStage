package org.firstinspires.ftc.teamcode.common.Commands.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;

public class IntakeStopCommand extends SequentialCommandGroup {
    public IntakeStopCommand() {
        super(
                new InstantCommand(() -> Intake.stopIntaking())
        );
    }
}
