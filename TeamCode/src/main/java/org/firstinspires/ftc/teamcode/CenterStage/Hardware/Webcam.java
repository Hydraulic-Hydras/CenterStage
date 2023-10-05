package org.firstinspires.ftc.teamcode.CenterStage.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class Webcam extends Contraption {

    private OpenCvCamera camera;
    private WebcamName webcamName;
    private String deviceName;
    private Telemetry telemetry;
    private static final int WIDTH = 800;
    private static final int HEIGHT = 448;

    public Webcam(LinearOpMode opMode, String deviceName) {
        this.opMode = opMode;
        this.deviceName = deviceName;
    }

    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        webcamName = hwMap.get(WebcamName.class, deviceName);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);

                FtcDashboard dashboard = FtcDashboard.getInstance();
                telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
                FtcDashboard.getInstance().startCameraStream(camera, 30);
                telemetry.update();

                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {

            }

        });

        camera.showFpsMeterOnViewport(true);

    }

    public void stopStreaming() {
        camera.stopStreaming();
    }
    public void setPipeline(OpenCvPipeline pipeline) {
        camera.setPipeline(pipeline);
    }

}
