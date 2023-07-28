package com.roboracers.gaeldrive.utils;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class VectorUtils {

    public static RealVector Vector2dToVector (Vector2d vector2d) {
        return new ArrayRealVector(new double[] {vector2d.getX(), vector2d.getY()});
    }
    public static double CrossProduct2d (RealVector t1, RealVector t2) {
        return t1.getEntry(0)*t2.getEntry(1)-t1.getEntry(1)*t2.getEntry(0);
    }
}
