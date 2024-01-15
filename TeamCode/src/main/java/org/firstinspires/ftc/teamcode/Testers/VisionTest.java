package org.firstinspires.ftc.teamcode.Testers;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStage.PropVision;
import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp (group = "Testers")
public class VisionTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private PropVision propVision = new PropVision(this.telemetry, true);

    public DigitalChannel LED_GreenL;
    public DigitalChannel LED_RedL;
    public DigitalChannel LED_GreenR;
    public DigitalChannel LED_RedR;

    public DigitalChannel ALED_Green;
    public DigitalChannel ALED_Red;
    public DigitalChannel BLED_Green;
    public DigitalChannel BLED_Red;

    Side side;
    @Override
    public void runOpMode() {

        initCam();
        initLEDS();

        while (!isStarted()) {
            side = propVision.getSide();
            telemetry.addData("Side: ", side);
            telemetry.update();
        }

        side = propVision.getSide();
        telemetry.addData("Side: ", side);
        telemetry.update();

        // close camera
        visionPortal.close();

        if (isStopRequested()) return;

        if (side == Side.RIGHT) {
            ALED_Green.setState(true);
            ALED_Red.setState(true);
            BLED_Green.setState(true);
            BLED_Red.setState(true);
            LED_GreenL.setState(true);
            LED_RedL.setState(true);
            LED_GreenR.setState(false);
            LED_RedR.setState(false);
        }  else if (side == Side.CENTER) {
            ALED_Green.setState(false);
            ALED_Red.setState(false);
            BLED_Green.setState(false);
            BLED_Red.setState(false);
            LED_GreenL.setState(true);
            LED_RedL.setState(true);
            LED_GreenR.setState(true);
            LED_RedR.setState(true);
        }  else {
            ALED_Green.setState(true);
            ALED_Red.setState(true);
            BLED_Green.setState(true);
            BLED_Red.setState(true);
            LED_GreenL.setState(false);
            LED_RedL.setState(false);
            LED_GreenR.setState(true);
            LED_RedR.setState(true);
        }


    }


    private void initCam() {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(0, 0));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(propVision);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(propVision, true);
    }

    private void initLEDS() {
        LED_GreenL = hardwareMap.get(DigitalChannel.class, "LED_Green-L");
        LED_GreenL.setMode(DigitalChannel.Mode.OUTPUT);

        LED_RedL = hardwareMap.get(DigitalChannel.class, "LED_Red-L");
        LED_RedL.setMode(DigitalChannel.Mode.OUTPUT);

        LED_GreenR = hardwareMap.get(DigitalChannel.class, "LED_Green-R");
        LED_GreenR.setMode(DigitalChannel.Mode.OUTPUT);

        LED_RedR = hardwareMap.get(DigitalChannel.class, "LED_Red-R");
        LED_RedR.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Green = hardwareMap.get(DigitalChannel.class, "ALED_Green");
        ALED_Green.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Red = hardwareMap.get(DigitalChannel.class, "ALED_Red");
        ALED_Red.setMode(DigitalChannel.Mode.OUTPUT);

        BLED_Green = hardwareMap.get(DigitalChannel.class, "BLED_Green");
        BLED_Green.setMode(DigitalChannel.Mode.OUTPUT);

        BLED_Red = hardwareMap.get(DigitalChannel.class, "BLED_Red");
        BLED_Red.setMode(DigitalChannel.Mode.OUTPUT);
    }
}
