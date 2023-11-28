package org.firstinspires.ftc.teamcode.CenterStage;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelPipeline extends OpenCvPipeline {

    Mat hsvFrame = new Mat();
    Mat hierarchy = new Mat();
    Mat whiteMask = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Mat whiteMask = preprocessFrame(input);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();


        return input;
    }


    private Mat preprocessFrame(Mat frame) {
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowWhite = new Scalar(0,0,0);
        Scalar highWhite = new Scalar(0,0,0);

        Core.inRange(hsvFrame, lowWhite, highWhite, whiteMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(whiteMask, whiteMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(whiteMask, whiteMask, Imgproc.MORPH_CLOSE, kernel);

        return whiteMask;

    }
}
