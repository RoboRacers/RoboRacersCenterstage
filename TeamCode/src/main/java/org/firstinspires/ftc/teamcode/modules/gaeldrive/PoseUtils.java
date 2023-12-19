package org.firstinspires.ftc.teamcode.modules.gaeldrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.roboracers.gaeldrive.utils.MismatchedLengthException;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class PoseUtils {

    /**
     * Util method to convert a Pose2d object to a RealVector object.
     *
     * @param pose A Pose2d Object
     * @return A RealVector Object
     */
    public static RealVector poseToVector(Pose2d pose) {
        return new ArrayRealVector(new double[] {pose.getX(), pose.getY(), pose.getHeading()});
    }

    /**
     * Util method to convert a RealVector object to a Pose2d object.
     *
     * @param vector A RealVector object
     * @return A Pose2d object
     */
    public static Pose2d vectorToPose(RealVector vector) {
        if (vector.getDimension() != 3) {
            throw new RuntimeException("Mismatched vector length in vectorToPose");
        }
        return new Pose2d(vector.getEntry(0), vector.getEntry(1), vector.getEntry(2));
    }

    /**
     * Converts the roadrunner Vector2d to an Array Real Vector
     * @param vector2d
     * @return Array Real Vector
     */
    public static RealVector Vector2dToVector (Vector2d vector2d) {
        return new ArrayRealVector(new double[] {vector2d.getX(), vector2d.getY()});
    }
}
