package org.firstinspires.ftc.teamcode.common.CommandBase;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand() {
        super(
                new WaitCommand(200),
                new InstantCommand(() -> Intake.rotateBucket.setPosition(Intake.POS_PANEL)),
                new WaitCommand(600),
                new InstantCommand(() -> Intake.rotateBucket.setPosition(Intake.POS_DUMP)),
                new WaitCommand(1000),
                new InstantCommand(() -> Intake.rotateBucket.setPosition(Intake.POS_REST))
        );
    }
}
