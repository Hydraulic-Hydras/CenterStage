package org.firstinspires.ftc.teamcode.common.Commands.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;

@Config
public class IntakeCommand extends ParallelCommandGroup {
    public IntakeCommand() {
        super(
                new InstantCommand(Intake::startIntaking),
                new WaitCommand(1000),
                new InstantCommand(Intake::stopIntaking)

        );

    }

}
