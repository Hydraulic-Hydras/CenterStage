package org.firstinspires.ftc.teamcode.common.Hardware.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Config
public class TFOD extends Contraption {

    public static VisionPortal myVisionPortal;
    public PropVision propVision;

    public boolean USE_WEBCAM;

    public TFOD(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    @Override
    public void initialize(HardwareMap hwMap) {
        USE_WEBCAM = true;

        VisionPortal.Builder myVisionPortalBuilder;
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(propVision);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    public void addProcessor(VisionProcessor visionProcessor) {
        TFOD.myVisionPortal.setProcessorEnabled(visionProcessor, true);
    }

    public void close() {
        TFOD.myVisionPortal.close();
    }
}
