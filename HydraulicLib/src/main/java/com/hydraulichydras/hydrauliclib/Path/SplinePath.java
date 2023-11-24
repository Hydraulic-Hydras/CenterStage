package com.hydraulichydras.hydrauliclib.Path;

import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Geometry.Spline;
import com.hydraulichydras.hydrauliclib.Geometry.Vector2D;

import java.util.ArrayList;
public class SplinePath {
    public SplineInterpolator interpolator;
    private final ArrayList<SplinePose> controlPoses = new ArrayList<>();

    public SplinePath() {
        interpolator = new SplineInterpolator();
    }

    public SplinePath addPose(double x, double y, Vector2D tangent) {
        this.addPose(new SplinePose(x, y, tangent));
        return this;
    }

    public SplinePath addPose(double x, double y, double h, double m) {
        return this.addPose(new SplinePose(x, y, Vector2D.fromHeadingAndMagnitude(h, m)));
    }

    public SplinePath addPose(double x, double y, double h) {
        return this.addPose(new SplinePose(x, y, Vector2D.fromHeadingAndMagnitude(h, 1)));
    }

    public SplinePath addPose(SplinePose pose) {
        this.controlPoses.add(pose);
        return this;
    }

    public SplinePath flip() {
        for (SplinePose pose : controlPoses) {
            double x = pose.x;
            pose.x = pose.y;
            pose.y = x;

            double v_x = pose.tangent.x;
            pose.tangent.x = pose.tangent.y;
            pose.tangent.y = v_x;
        }
        return this;
    }

    public SplinePath negateX() {
        for (SplinePose pose : controlPoses) {
            pose.x *= -1;
        }
        return this;
    }

    public SplinePath negateY() {
        for (SplinePose pose : controlPoses) {
            pose.y *= -1;
        }
        return this;
    }

    public SplinePath offsetX(double x) {
        for (SplinePose pose : controlPoses) {
            pose.x += x;
        }
        return this;
    }

    public SplinePath offsetY(double y) {
        for (SplinePose pose : controlPoses) {
            pose.y += y;
        }
        return this;
    }

    public SplinePath construct() {
        if (controlPoses.size() <= 1)
            throw new IllegalStateException("Need a minimum of two control poses.");
        interpolator = new SplineInterpolator();
        interpolator.setControlPoses(controlPoses);
        return this;
    }

    public Pose get(double t, int n) {
        if (t <= 0) {
            return startPose();
        } else if (t >= controlPoses.size()) {
            return endPose();
        } else {
            return interpolator.get(t, n);
        }
    }

    public double getHeading(double t) {
        return interpolator.getHeading(t);
    }

    public double curvature(double t) {
        return interpolator.curvature(t);
    }

    public Spline getSpline(double t) {
        return interpolator.getSpline(t);
    }

    public ArrayList<Spline> getSplines() {
        return interpolator.getSplines();
    }

    public int length() {
        return controlPoses.size() - 1;
    }

    public Pose startPose() {
        return controlPoses.get(0).pose();
    }

    public Pose endPose() {
        return controlPoses.get(length()).pose();
    }
}
