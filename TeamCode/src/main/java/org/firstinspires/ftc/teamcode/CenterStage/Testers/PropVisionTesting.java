package org.firstinspires.ftc.teamcode.CenterStage.Testers;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStage.common.Vision.PropVision;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class PropVisionTesting extends LinearOpMode {

    private VisionPortal visionPortal;
    private PropVision propVision = new PropVision(this.telemetry,true);

    @Override
    public void runOpMode() throws InterruptedException {
        initPropVision();
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){

                telemetry.update();
                sleep(20);

            }

        }

        visionPortal.close();

    }
    private void initPropVision(){
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(propVision);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(propVision, true);
        propVision.onDrawFrame(new Canvas(), 320, 240, 100, 100, telemetry);

    }
}
