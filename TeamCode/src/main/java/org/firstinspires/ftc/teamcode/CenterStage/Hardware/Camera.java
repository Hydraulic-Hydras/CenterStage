package org.firstinspires.ftc.teamcode.CenterStage.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera extends Contraption {

    private WebcamName webcamName;
    private OpenCvCamera camera;
    private String deviceName;

    private static final int WIDTH = 640;
    private static final int HEIGHT = 360;

    public Camera(LinearOpMode opMode, String deviceName) {
        this.opMode = opMode;
        this.deviceName = deviceName;
    }

    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hwMap.appContext.getPackageName());

        webcamName = hwMap.get(WebcamName.class, deviceName);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT); // 864, 480
                FtcDashboard.getInstance().startCameraStream(camera, 30);
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

