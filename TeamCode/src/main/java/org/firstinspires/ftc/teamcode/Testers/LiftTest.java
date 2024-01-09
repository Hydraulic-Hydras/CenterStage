package org.firstinspires.ftc.teamcode.Testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.CommandBase.LiftCommand;
import org.firstinspires.ftc.teamcode.common.CommandBase.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
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

                        new LiftCommand(1300, 1),

                        new OuttakeCommand()


                ));
    }

    @Override
    public void run() {
        super.run();

        mitsumi.telemetry(telemetry);
        telemetry.update();
    }
}
