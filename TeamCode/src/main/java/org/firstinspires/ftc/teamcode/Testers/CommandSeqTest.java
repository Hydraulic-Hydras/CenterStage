package org.firstinspires.ftc.teamcode.Testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.CommandBase.ScoringCommand;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Globals;

@Disabled
@Autonomous
public class CommandSeqTest extends CommandOpMode {

    public Mitsumi mitsumi = new Mitsumi(this);
    public Intake intake = new Intake(this);
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        Globals.IS_AUTO = true;

        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();

        intake.initialize(hardwareMap);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ScoringCommand()
                )
        );

    }
    @Override
    public void run() {
        super.run();
    }
}
