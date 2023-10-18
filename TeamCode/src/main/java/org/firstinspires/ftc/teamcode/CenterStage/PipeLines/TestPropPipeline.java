package org.firstinspires.ftc.teamcode.CenterStage.PipeLines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class TestPropPipeline extends OpenCvPipeline {

    public static double cX = 0;
    public static double cY = 0;
    double width = 0;
    Mat hierarchy = new Mat();
    Mat hsvFrame = new Mat();
    Mat TeamProp = new Mat();

    public static double Scalar_common = 150; // For lower red = grey on colorizer
    public static double Scalar1 = 255; // Upper red values
    public static double Scalar2 = 255; // RGB format
    public static double Scalar3 = 255;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    // private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    public static final double FOV = 60;

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

    public void displayInfo(Telemetry telemetry) {
        telemetry.addData("Target IMU Angle", getAngleTarget(cX));
        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
        telemetry.addData("Distance in Inch", (getDistance(width)));
        telemetry.update();
    }
    private Mat preprocessFrame(Mat frame) {
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Scalar lowerRed = new Scalar(Scalar_common, Scalar_common, Scalar_common);
        Scalar upperRed = new Scalar(Scalar1, Scalar2, Scalar3);

        // changed from yellowMask to blue Mask
        Core.inRange(hsvFrame, lowerRed, upperRed, TeamProp);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(TeamProp, TeamProp, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(TeamProp, TeamProp, Imgproc.MORPH_CLOSE, kernel);

        return TeamProp;

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

    public static double getAngleTarget(double objMidpoint){
        double midpoint = -((objMidpoint - (CAMERA_WIDTH/2))*FOV)/CAMERA_WIDTH;
        return midpoint;
    }

    public static double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    public static double getDistance(double width) {
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return (distance + 5);
    }
}
