package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous
public class CameraTest extends LinearOpMode {

    OpenCvCamera camera;
    double cX = 0;
    double cY = 0;
    double width = 0;
    private static final int WIDTH = 640; // 640 or 800
    private static final int HEIGHT = 480;  // 360 or 448
    private static final double FOV = 60;

    public static double Scalar1_1 = 150; // For lower red = grey on colorizer
    public static double Scalar1_2 = 150;
    public static double Scalar1_3 = 150;

    public static double Scalar2_1 = 255; // Upper red values
    public static double Scalar2_2 = 255; // RGB format
    public static double Scalar2_3 = 255;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 822;  // Replace with the focal length of the camera in pixels

    // 728

    @Override
    public void runOpMode() {

        initOpenCV();

        waitForStart();

        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
        telemetry.addData("Distance in Inch", (getDistance(width)));
        telemetry.update();

        while (opModeIsActive()) {

            telemetry.addData("Target IMU Angle", getAngleTarget(cX));
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection

        }

        // Release resources
        camera.stopStreaming();
    }

    public void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT); // removed camera rotation here

                FtcDashboard dashboard = FtcDashboard.getInstance();
                telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
                FtcDashboard.getInstance().startCameraStream(camera, 30);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // set pipeline below
        camera.setPipeline(new ObjectDetectionPipeline());
    }


    class ObjectDetectionPipeline extends OpenCvPipeline {
        Mat hierarchy = new Mat();
        Mat hsvFrame = new Mat();
        Mat yellowMask = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);


            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            hierarchy.release();
            hsvFrame.release();
            yellowMask.release();

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

            // RGB FORMAT
            // Scalar lowerYellow = new Scalar(100, 100, 100);
            // Scalar upperYellow = new Scalar(180, 255, 255);

            // FOR BLUE
            // Scalar upperYellow = new Scalar(35, 220, 240);
            // Scalar lowerYellow = new Scalar(0, 80, 100);

            // FOR GREEN AND YELLOW
           // Scalar upperYellow = new Scalar(90, 2555, 255);
            // Scalar lowerYellow = new Scalar(50, 100, 50);

            Scalar lowerRed = new Scalar(Scalar1_1, Scalar1_2, Scalar1_3);
            Scalar upperRed = new Scalar(Scalar2_1, Scalar2_2, Scalar2_3);

            // Scalar lowerRed = new Scalar(150, 150, 150); // grey
            // Scalar upperRed = new Scalar(235, 100, 100); // the upper hsv threshold for your detection

            // changed from yellowMask to blue Mask
            Core.inRange(hsvFrame, lowerRed, upperRed, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;

        }


        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    public double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return (distance +  5);
    }
    private static double getAngleTarget(double objMidpoint){
        double midpoint = -((objMidpoint - (WIDTH/2))*FOV)/WIDTH;
        return midpoint;
    }

    public interface bgrColor {
        Scalar GREEN = new Scalar(0, 255, 0);
        Scalar RED = new Scalar(0, 0, 255);
        Scalar BLUE = new Scalar(255, 0, 0);
        Scalar VULCAN = new Scalar(12, 21, 140);
        Scalar PINK = new Scalar(255, 0, 255);
        Scalar PURPLE = new Scalar(255, 51, 51);
        Scalar CYAN = new Scalar(255, 255, 0);
        Scalar YELLOW = new Scalar(0, 255, 255);
        Scalar FOREST_GREEN = new Scalar(0, 102, 51);
        Scalar ORANGE = new Scalar(0, 128, 255);

        Scalar WHITE = new Scalar(255, 255, 255);
        Scalar BLACK = new Scalar(0, 0, 0);
        Scalar GRAY = new Scalar(96, 96, 96);

        Scalar PASTEL_RED = new Scalar(204, 204, 255);
        Scalar PASTEL_YELLOW = new Scalar(204, 255, 255);
        Scalar PASTEL_GREEN = new Scalar(204, 255, 204);
        Scalar PASTEL_BLUE = new Scalar(255, 255, 204);
        Scalar PASTEL_PURPLE = new Scalar(255, 204, 204);
        Scalar PASTEL_PINK = new Scalar(229, 204, 255);

    }
}

