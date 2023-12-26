package org.firstinspires.ftc.teamcode.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Localization.Drivetrain;
import com.hydraulichydras.hydrauliclib.Localization.Localizer;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Pathing.ChaiTrackingConfig;
import org.firstinspires.ftc.teamcode.common.Pathing.ChaiTrackingPath;

@Config
public class ChaiTrackingCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private final ChaiTrackingPath chaiTrackingPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;

    public static double xP = 0.0385;
    public static double xD = 0.006;

    public static double yP = 0.0385;
    public static double yD = 0.006;

    public static double hP = 1;
    public static double hD = 0.04;

    public static double kStatic = 0.05;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    private RobotHardware robot = RobotHardware.getInstance();

    public ChaiTrackingCommand(Drivetrain thedrivetrain, Localizer thelocalizer, ChaiTrackingPath ThechaiTrackingPath) {
        this.drivetrain = thedrivetrain;
        this.localizer = thelocalizer;
        this.chaiTrackingPath = ThechaiTrackingPath;
        this.endPose = chaiTrackingPath.getEndPose();
    }

    @Override
    public void execute() {
        if (chaiTrackingPath.isFinished()) PID = true;

        Pose robotPose = localizer.getPos();
        Pose targetPose = chaiTrackingPath.update(robotPose);

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < ChaiTrackingConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < ChaiTrackingConfig.ALLOWED_HEADING_ERROR) finished = true;


        if (PID) {
            Pose delta = targetPose.subtract(robotPose);

            double xPower = xController.calculate(robotPose.x, targetPose.x);
            double yPower = yController.calculate(robotPose.y, targetPose.y);
            double hPower = -hController.calculate(0, delta.heading);

            double x_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
            double y_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);

            if (Math.abs(x_rotated) < 0.01) x_rotated = 0;
            else x_rotated += kStatic * Math.signum(x_rotated);
            if (Math.abs(y_rotated) < 0.01) y_rotated = 0;
            else y_rotated += kStatic * Math.signum(y_rotated);
            if (Math.abs(hPower) < 0.01) hPower = 0;
            else hPower += kStatic * Math.signum(hPower);

            drivetrain.set(new Pose((y_rotated / robot.getVoltage() * 12.5) *1.6, x_rotated / robot.getVoltage() * 12.5, hPower / robot.getVoltage() * 12.5));
        } else {
            Pose delta = targetPose.subtract(robotPose);
            double y_rotated = delta.x * Math.cos(robotPose.heading) - delta.y * Math.sin(robotPose.heading);
            double x_rotated = delta.x * Math.sin(robotPose.heading) + delta.y * Math.cos(robotPose.heading);

            double xPercentage = x_rotated / chaiTrackingPath.getRadius() * ChaiTrackingConfig.FOLLOW_SPEED;
            double yPercentage = y_rotated / chaiTrackingPath.getRadius() * ChaiTrackingConfig.FOLLOW_SPEED;
            double hPower = -hController.calculate(0, delta.heading);

            drivetrain.set(new Pose(xPercentage *1.6, yPercentage, hPower));
        }
    }

    @Override
    public boolean isFinished() {
        return PID && finished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }
}
