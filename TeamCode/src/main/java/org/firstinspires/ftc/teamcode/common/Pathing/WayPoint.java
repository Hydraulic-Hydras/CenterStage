package org.firstinspires.ftc.teamcode.common.Pathing;

import com.hydraulichydras.hydrauliclib.Geometry.Point;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;

import java.util.Locale;

public class WayPoint {

    private final WayPoint.Type type;
    private final Point point;
    private final double radius;

    public WayPoint(Point point, double radius) {
        this.type = point instanceof Pose ? Type.POSE : Type.POINT;
        this.point = point;
        this.radius = radius;
    }

    public WayPoint.Type getType() {
        return this.type;
    }

    public Point getPoint() {
        return this.point;
    }

    public double getRadius() {
        return this.radius;
    }

    public enum Type {
        POINT, POSE
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%s %s %.2f", getType(), getPoint(), getRadius());
    }
}
