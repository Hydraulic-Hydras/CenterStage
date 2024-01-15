package org.firstinspires.ftc.teamcode.common.Util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Mitsumi;

@Config
public class TeleOpInfo {

    Telemetry telemetry;
    public TeleOpInfo(Telemetry telemetry) {
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

        /*
        /** removed Distance sensor telem because OpMode's became too delayed
        telemetry.addData("Backdrop Distance", Double.parseDouble(
                JavaUtil.formatNumber(LEDS.distanceBackdrop.getDistance(DistanceUnit.CM), 0)));
        telemetry.addLine();
         */

        telemetry.update();
    }
}
