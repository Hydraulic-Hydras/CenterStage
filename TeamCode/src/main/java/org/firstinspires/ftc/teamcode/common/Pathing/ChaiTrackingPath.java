package org.firstinspires.ftc.teamcode.common.Pathing;

import com.hydraulichydras.hydrauliclib.Geometry.Point;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Collections;
import java.util.LinkedList;

public class ChaiTrackingPath {
    private LinkedList<WayPoint> waypoints = new LinkedList<>();
    private int targetIdx = 1;
    private boolean finished;

    public ChaiTrackingPath(WayPoint... ws) {
        if (ws.length < 2) throw new IllegalArgumentException();
        Collections.addAll(waypoints, ws);
        if(waypoints.getLast().getType() != WayPoint.Type.POSE) throw new IllegalArgumentException();
    }

    public Pose update(Pose robot) {
        WayPoint prev = waypoints.get(targetIdx - 1);
        WayPoint target = waypoints.get(targetIdx);

        double distance = robot.distanceTo(target.getPoint());

        if(distance > target.getRadius()){
            Point intersection = ChaiTrackingUtil.lineCircleIntersection(
                    prev.getPoint(), target.getPoint(), robot, target.getRadius());
            Pose targetPose;

            if(target.getType() == WayPoint.Type.POSE){
                targetPose = new Pose(intersection, ((Pose)target.getPoint()).heading);
            }else{
                double robotAngle = AngleUnit.normalizeRadians(robot.heading);
                double forwardAngle = intersection.subtract(robot).atan() - (Math.PI/2);
                double backwardsAngle = AngleUnit.normalizeRadians(forwardAngle + Math.PI);

                double autoAngle =
                        Math.abs(AngleUnit.normalizeRadians(robotAngle - forwardAngle)) <
                                Math.abs(AngleUnit.normalizeRadians(robotAngle - backwardsAngle)) ?
                                forwardAngle : backwardsAngle;

                targetPose = new Pose(intersection, autoAngle);
            }

            return targetPose;
        }else{
            if(targetIdx == waypoints.size() - 1){
                finished = true;
                return getEndPose();
            }else{
                targetIdx++;
                return update(robot);
            }
        }
    }

    public boolean isFinished(){
        return finished;
    }

    public Pose getEndPose(){
        return (Pose) waypoints.getLast().getPoint();
    }

    public double getRadius(){
        return waypoints.get(targetIdx).getRadius();
    }
}
