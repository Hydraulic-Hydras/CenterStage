package org.firstinspires.ftc.teamcode.CenterStage.common.Vision;

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

public class TeamPropPipeline extends OpenCvPipeline {

    Mat hierarchy = new Mat();
    Mat hsvFrame = new Mat();
    Mat teamMask = new Mat();

    Telemetry telemetry;

    double cX = 0;
    double cY = 0;
    double width = 0;

    int objectPos = 0;

    boolean ObjectFound;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 0.7;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    @Override
    public Mat processFrame(Mat input) {

        Mat teamMask = preprocessFrame(input);

        // Find contours of the detected team prop
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(teamMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 0), 2);
            // Calculate the width of the bounding box
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
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
        teamMask.release();

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        // Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2RGB);
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);
        
        Scalar lowerBlue = new Scalar(100, 100, 100);
        Scalar upperBlue = new Scalar(180, 255, 255);

        Scalar lowerRed = new Scalar(155, 155, 155);
        Scalar upperRed = new Scalar(255, 255, 255);


        Core.inRange(hsvFrame, lowerRed, upperRed, teamMask);
        Core.inRange(hsvFrame, lowerBlue, upperBlue, teamMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        Imgproc.morphologyEx(teamMask, teamMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(teamMask, teamMask, Imgproc.MORPH_CLOSE, kernel);

        return teamMask;
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

    public void telemetry(Telemetry telemetry) {
        processFrame(teamMask);

        if (getDistance(width) >= 42 ) {
            // Middle
            telemetry.addData("Object is in the Middle", ObjectFound = true);
            telemetry.addData("Object's Position: 2", objectPos = 2);
            telemetry.update();

        }   else if (getDistance(width) >= 18) {
            // Left
            telemetry.addData("Object is on the Left", ObjectFound = true);
            telemetry.addData("Object's Position: 1", objectPos = 1);
            telemetry.update();

        }   else {
            // Right
            telemetry.addData("No Object detected, object should be on the Right", ObjectFound = false);
            telemetry.addData("Object's Position: 3", objectPos = 3);
            telemetry.update();
        }
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    public double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return (distance);
    }
}
