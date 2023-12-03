package org.firstinspires.ftc.teamcode.Testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStage.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PipeLineTest extends CommandOpMode {

    private PropPipeline propPipeline;
    private VisionPortal portal;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(propPipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (!isStarted()) {
            telemetry.addLine("Initialized");
            telemetry.addData("Side: ", propPipeline.getLocation());
            telemetry.update();
        }

        portal.close();
    }

    @Override
    public void run() {
    super.run();

    }
}
