package com.roboracers.gaeldrive.distance;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.sin;
import static java.lang.Double.NaN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.roboracers.gaeldrive.utils.VectorUtils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class FieldDistance {

    public static List<Vector2d[]> FieldGeometry = new ArrayList<>();

    static  {
        FieldGeometry.add(new Vector2d[] {new Vector2d(-72, 72), new Vector2d(72, 72)});
        FieldGeometry.add(new Vector2d[] {new Vector2d(72, 72), new Vector2d(72, -72)});
        FieldGeometry.add(new Vector2d[] {new Vector2d(72, -72), new Vector2d(-72, -72)});
        FieldGeometry.add(new Vector2d[] {new Vector2d(-72, -72), new Vector2d(72, -72)});
    }

    public static double calculateSimulatedDistance(Pose2d origin) {

        List<Double> distances = new ArrayList<>();

        for(Vector2d[] line : FieldGeometry) {
            double distance = getRayToLineSegmentIntersection(origin, line[0], line[1]);
            if (distance != -1 ) {
                distances.add(distance);
            }
        }

        if (distances.isEmpty() == false) {
            return Collections.min(distances);
        }
        return 0;
    }


    public static double getRayToLineSegmentIntersection(Pose2d pose, Vector2d point1, Vector2d point2) {

        Vector2d rayOrigin = new Vector2d(pose.getX(), pose.getY());
        Vector2d v1 = rayOrigin.minus(point1);
        Vector2d v2 = point2.minus(point1);
        Vector2d v3 = new Vector2d(-sin(pose.getHeading()), cos(pose.getHeading()));

        double dot = v2.dot(v3);

        if (abs(dot) < 0.000001) {
            return -1;
        }

        double t1 = VectorUtils.CrossProduct2d(VectorUtils.Vector2dToVector(v2), VectorUtils.Vector2dToVector(v1))/dot;
        double t2 = (v1.dot(v3)) / dot;

        if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0)) {
            return t1;
        }

        return -1;
    }

}
