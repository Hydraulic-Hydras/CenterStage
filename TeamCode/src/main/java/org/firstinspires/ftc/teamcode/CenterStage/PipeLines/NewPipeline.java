package org.firstinspires.ftc.teamcode.CenterStage.PipeLines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class NewPipeline extends OpenCvPipeline {

    public enum Side {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Side side;
    Mat mat = new Mat();

    // Full screen size
    // x = 800
    // y = 448
    //
    //                   +Y axis (228)
    //
    //                      |
    //                      |
    //                      |
    //                      |
    //                      |
    //   (-400) ------------|------------  +X axis (400)
    //                      |
    //                      |
    //                      |
    //                      |
    //                      |
    //                      |
    //
    //                   (-228)
    private static final int WIDTH = 800;
    private static final int HEIGHT = 448;
    Rect LEFT_ROI = new Rect(
            new Point(-350, -50),
            new Point(-250, -100)
    );
    Rect MIDDLE_ROI = new Rect(
            new Point(-100, 50),
            new Point(50, -100)
    );
    Rect ROI_NOTFOUND;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

                            //      H   S   V
        Scalar lowRed = new Scalar(244, 17, 95);
        Scalar highRed = new Scalar(255, 36, 0);
        Mat redMask = new Mat();

        Core.inRange(mat, lowRed, highRed, redMask);

                             //      H    S    V
        Scalar lowBlue = new Scalar(178, 255, 255);
        Scalar highBlue = new Scalar(0, 48, 143);
        Mat blueMask = new Mat();

        Core.inRange(mat, lowBlue, highBlue, blueMask);

        redMask.release();
        blueMask.release();

        Scalar colorLeft = new Scalar(255, 0, 255);
        Scalar colorMiddle = new Scalar(0, 255, 0);
        Scalar colorNotFound = new Scalar(255, 0, 0);


        if (side == Side.LEFT) {
            Imgproc.rectangle(input, LEFT_ROI, colorLeft, 5);

        }   else if (side == Side.MIDDLE) {
            Imgproc.rectangle(input, MIDDLE_ROI, colorMiddle, 5);

        }   else if (side == Side.RIGHT) {
            Imgproc.rectangle(input, ROI_NOTFOUND, colorNotFound, 5);
        }


        mat.release();
        return input;
    }

    public Side getSide() {
        return side;
    }
}
