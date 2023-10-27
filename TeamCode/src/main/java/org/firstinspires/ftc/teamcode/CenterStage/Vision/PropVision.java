package org.firstinspires.ftc.teamcode.CenterStage.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PropVision implements VisionProcessor {

    // bounding lines for the three regions
    int height = 480;
    int width = 640;

    int LEFT_LINE = (width / 3);
    int RIGHT_LINE = (2 * (width / 3));

    boolean isRed;

    public enum PropLocation{
        LEFT,
        MIDDLE,
        RIGHT
    }

    public PropLocation location;
    static final Scalar GREEN = new Scalar(0, 255, 0);

    Telemetry telemetry;

    public PropVision(Telemetry telemetry, boolean isRed) {
        this.telemetry = telemetry;
        this.isRed = isRed;
    }

    Mat mat = new Mat();
    Mat thresh = new Mat();

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // Convert to HSV format
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(0,0,0);
        Scalar highHSV = new Scalar(255,255,255);

        // if isRed = true
        if(isRed) {
            lowHSV = new Scalar(160, 50, 50); // lower bound HSV for red
            highHSV = new Scalar(180, 255, 255); // higher bound HSV for red
        } else {
            // if isRed = false
            lowHSV = new Scalar(110, 50, 50); // lower bound HSV for for blue
            highHSV = new Scalar(120, 255, 255); // higher bound HSV for blue

        }

        Core.inRange(mat,lowHSV,highHSV,thresh);
        Mat left = thresh.submat(0,height,0,LEFT_LINE);
        Mat center = thresh.submat(0,height,LEFT_LINE,RIGHT_LINE);
        Mat right = thresh.submat(0,height,RIGHT_LINE,width);

        // draw lines to make the team prop fit inside the lines
        Imgproc.line(frame,new Point(LEFT_LINE,0), new Point(LEFT_LINE,height),GREEN,4);
        Imgproc.line(frame,new Point(RIGHT_LINE,0), new Point(RIGHT_LINE,height),GREEN,4);


        int leftNum = Core.countNonZero(left);
        int middleNum = Core.countNonZero(center);
        int rightNum = Core.countNonZero(right);

        if (leftNum > middleNum && leftNum > rightNum){
            location = PropLocation.LEFT;
        } else if (middleNum > leftNum && middleNum > rightNum){
            location = PropLocation.MIDDLE;
        } else if (rightNum > middleNum && rightNum > leftNum){
            location = PropLocation.RIGHT;
        }

        telemetry.addLine("LEFT: " + leftNum);
        telemetry.addLine("CENTER: " + middleNum);
        telemetry.addLine("RIGHT: " + rightNum);
        telemetry.addLine("Location: " + location);
        telemetry.update();

        // our camera output gets put back into the frame - showing which pixels are being used
        frame.copyTo(frame);
        // be responsible with memory
        mat.release();
        thresh.release();
        left.release();
        right.release();
        center.release();

        return null;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.FILL);

        int myFontSize = 12;

        double relation = Math.sqrt(canvas.getWidth() * canvas.getHeight());
        relation = relation / 250;
        paint.setTextSize((float) (myFontSize * relation));

        canvas.drawText("Location: " + location, 670, 400, paint);

        }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
}