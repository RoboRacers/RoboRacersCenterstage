package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorBuffer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.PoseUtils;

/**
 * A motion model that uses the tracking wheel estimate to estimate the movement of a robot in a given timeframe.
 * Used in Monte Carlo Localization
 * @see MotionModel
 */
public class TrackingWheelMotionModel implements MotionModel{

    RealVector prevState;
    RealVector currentState;

    public TrackingWheelMotionModel(Pose2d startPose) {
        currentState = PoseUtils.poseToVecor(startPose);
    }

    /**
     * Gets the translation in state using the delta in the tracking wheel pose.
     * @return Translational vector
     */
    @Override
    public  RealVector getTranslationVector() {
        prevState = currentState;
        currentState = PoseUtils.poseToVecor(SensorBuffer.trackingWheelPose);
        RealVector translation = currentState.subtract(prevState);

        return translation;
    }
}
