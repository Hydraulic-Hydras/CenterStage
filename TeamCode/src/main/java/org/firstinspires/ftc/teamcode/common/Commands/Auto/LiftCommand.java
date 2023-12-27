package org.firstinspires.ftc.teamcode.common.Commands.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;

@Config
public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(int Target, double power) {
        super(
                new WaitCommand(500),
                new InstantCommand(() -> Mitsumi.LeftCascade.setTargetPosition(Target)),
                new InstantCommand(() -> Mitsumi.RightCascade.setTargetPosition(Target)),
                new InstantCommand(() -> Mitsumi.LeftCascade.setPower(power)),
                new InstantCommand(() -> Mitsumi.RightCascade.setPower(power)),

                new InstantCommand(() -> Mitsumi.LeftCascade.setMode(DcMotor.RunMode.RUN_TO_POSITION)),
                new InstantCommand(() -> Mitsumi.RightCascade.setMode(DcMotor.RunMode.RUN_TO_POSITION))

        );
    }
}
