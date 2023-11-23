package com.hydraulichydras.hydrauliclib.Geometry;

/**
 * MathUtils is an utility class that contains various mathematical functions and operations.
 */
public class MathUtils {

    public static double norm(double angle){
        return angle%(2*Math.PI);
    }
    public static double normDelta(double angle){
        return (angle+Math.PI)%(2*Math.PI)-Math.PI;
    }
    public static double max(double... args){
        double max = args[0];
        for(double d : args){
            if(d > max) max = d;
        }
        return max;
    }
}
