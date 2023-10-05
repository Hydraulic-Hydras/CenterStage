package org.firstinspires.ftc.teamcode.CenterStage.PipeLines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CustomPipeline extends OpenCvPipeline {
    Mat hsvFrame = new Mat();
    Mat redChannel = new Mat();
    Mat blueChannel = new Mat();
    Mat blueThreshold = new Mat();
    Mat redThreshold = new Mat();
    Mat binaryMat = new Mat();

    Rect blueRect, redRect;

    private List<MatOfPoint> redContours = new ArrayList<>();
    private List<MatOfPoint> blueContours = new ArrayList<>();
    private MatOfPoint biggestBlueContour;
    private MatOfPoint biggestRedContour;

    public int minThreshold = 155;
    public int maxThreshold = 200;


    @Override
    public Mat processFrame(Mat input) {
        // Convert from BGR to HSV
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);
        // Core.inRange(hsvFrame lower, upper, binaryMat);

        Core.extractChannel(hsvFrame, redChannel, 1);
        Core.extractChannel(hsvFrame, blueChannel, 2);

        // Blue threshold
        Imgproc.threshold(blueChannel, blueThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
        // Red threshold
        Imgproc.threshold(redChannel, redThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);

        blueContours.clear();
        redContours.clear();

        Imgproc.findContours(blueThreshold, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(redThreshold, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // still need to work n research

        return input;
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

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    // make a getDistance method here

}
