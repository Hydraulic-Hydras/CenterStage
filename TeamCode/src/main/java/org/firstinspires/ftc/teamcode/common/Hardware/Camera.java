package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
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

public class Camera extends Contraption {

    private WebcamName webcamName;
    private OpenCvCamera camera;
    private String deviceName;

    private static int WIDTH = 640;
    private static int HEIGHT = 480;

    public Camera(LinearOpMode opMode, String deviceName) {
        this.opMode = opMode;
        this.deviceName = deviceName;
    }

    public Camera (String deviceName) {
        this.deviceName = deviceName;
    }

    public Camera (LinearOpMode opMode, String deviceName, boolean isC920) {
        this.opMode = opMode;
        this.deviceName = deviceName;

        if (isC920) {
            WIDTH = 1280;
            HEIGHT = 720;
        }

    }

    public void initialize(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hwMap.appContext.getPackageName());

        webcamName = hwMap.get(WebcamName.class, deviceName);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT); // 864, 480
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        camera.showFpsMeterOnViewport(true);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry.update();

    }

    public void stopStreaming() {
        camera.stopStreaming();
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        camera.setPipeline(pipeline);
    }


}

