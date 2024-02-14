package org.firstinspires.ftc.teamcode.common.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Localization.Drivetrain;

public class PositionCommand extends CommandBase {

    public Drivetrain drivetrain;
    public Pose targetPose;

}
