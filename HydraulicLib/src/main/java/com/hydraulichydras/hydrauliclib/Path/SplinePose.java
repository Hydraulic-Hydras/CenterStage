package com.hydraulichydras.hydrauliclib.Path;

import com.hydraulichydras.hydrauliclib.Geometry.Point;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.hydraulichydras.hydrauliclib.Geometry.Vector2D;

public class SplinePose {
    public double x, y;
    public Vector2D tangent;

    public SplinePose(double x, double y, Vector2D tangent) {
        this.x = x;
        this.y = y;
        this.tangent = tangent;
    }

    public SplinePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.tangent = new Vector2D(Math.cos(heading), Math.sin(heading));
    }

    public Point add(SplinePose other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point subt(SplinePose other) {
        return new Point(x - other.x, y - other.y);
    }

    public Pose pose() {
        return new Pose(x, y, Math.atan2(y, x));
    }

    @Override
    public String toString() {
        return String.format("%.2f, %.2f, %s", x, y, tangent);
    }
}