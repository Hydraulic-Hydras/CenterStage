package org.firstinspires.ftc.teamcode.common.Util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;
import org.firstinspires.ftc.teamcode.common.Hardware.Drive.Drivetrain;

@Config
public class InfoFiles {

    Telemetry telemetry;
    public InfoFiles(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void Telemetry(Telemetry telemetry) {
        telemetry.addData("Left Position: ", Mitsumi.LeftCascade.getCurrentPosition());
        telemetry.addData("Right Position: ", Mitsumi.RightCascade.getCurrentPosition());
        telemetry.addLine();

        telemetry.addData("Outtake State ", Intake.getState());
        telemetry.addLine();

        telemetry.addData("Drone Angle", Launcher.getAngle());
        telemetry.addData("Drone Trigger", Launcher.getState());
        telemetry.addLine();

        telemetry.addData("Backdrop Distance", Double.parseDouble(
                JavaUtil.formatNumber(Drivetrain.distanceBackdrop.getDistance(DistanceUnit.CM), 0)));
        telemetry.addLine();

        Drivetrain.drive.update();

        Pose2d poseEstimate = Drivetrain.drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        telemetry.update();
    }
}
