package org.firstinspires.ftc.teamcode.Testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Commands.Auto.LiftCommand;
import org.firstinspires.ftc.teamcode.common.Commands.Auto.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;

@Disabled
@Autonomous( name = "JAVA-LiftTest")
public class LiftTest extends CommandOpMode {

    private final Mitsumi mitsumi = new Mitsumi(this);
    private final Intake intake = new Intake(this);

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
     CommandScheduler.getInstance().reset();

        mitsumi.initialize(hardwareMap);
        mitsumi.autoInit();
        intake.initialize(hardwareMap);

        Intake.rotateBucket.setPosition(Intake.POS_REST);

        while (!isStarted()) {
            telemetry.addLine("Auto in init");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        new LiftCommand(1500, 0.9),

                        // **Switched with Lift Command loop
                        // new InstantCommand(() -> mitsumi.AutoMoveTo(1500, 0.9)),

                        // **Switched these commands to Outtake Command loop**
                        // new InstantCommand(() -> Intake.rotateBucket.setPosition(Intake.POS_DUMP)),
                        // new WaitCommand(1100),
                        // new InstantCommand(() -> Intake.rotateBucket.setPosition(Intake.POS_REST)),

                        new OuttakeCommand(),
                        new LiftCommand(0,0.55)

                        // new InstantCommand(() -> mitsumi.AutoMoveTo(0, 0.55))

                ));
    }

    @Override
    public void run() {
        super.run();

        mitsumi.telemetry(telemetry);
        telemetry.update();
    }
}
