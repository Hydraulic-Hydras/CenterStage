package org.firstinspires.ftc.teamcode.common.CommandBase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class ScoringCommand extends SequentialCommandGroup {

    public ScoringCommand() {
        super(
                new WaitCommand(500),
                new LiftCommand(1250, 0.85),
                new OuttakeCommand(),
                new WaitCommand(500),
                new LiftCommand(0, 0.65)
        );
    }
}
