package org.firstinspires.ftc.teamcode.common.CommandBase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class ScoringCommand extends SequentialCommandGroup {

    public ScoringCommand() {
        super(
                new LiftCommand(1250, 1),
                new OuttakeCommand(),
                new LiftCommand(-200, 1)
        );
    }
}
