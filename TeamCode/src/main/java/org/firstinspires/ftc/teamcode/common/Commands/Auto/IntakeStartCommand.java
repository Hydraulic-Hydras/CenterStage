package org.firstinspires.ftc.teamcode.common.Commands.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;

@Config
public class IntakeStartCommand extends InstantCommand {
    public IntakeStartCommand() {
        super(
                (() -> Intake.startIntaking())
        );

    }

}
