package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.roboracers.gaeldrive.motion.MotionModel;

import org.apache.commons.math3.linear.RealVector;
import com.roboracers.gaeldrive.utils.PoseUtils;

/**
 * A motion model that uses the tracking wheel estimate to estimate the movement of a robot in a given timeframe.
 * Used in Monte Carlo Localization
 * @see MotionModel
 */
public class TrackingWheelMotionModel implements MotionModel{

    ThreeTrackingWheelLocalizer trackingWheelLocalizer;

    Pose2d trackingWheelPose;

    RealVector prevState;
    RealVector currentState;


    public TrackingWheelMotionModel(Pose2d startPose, ThreeTrackingWheelLocalizer localizer) {
        currentState = PoseUtils.poseToVector(startPose);
        trackingWheelLocalizer = localizer;
    }

    public void update() {
        trackingWheelLocalizer.update();
        trackingWheelPose = trackingWheelLocalizer.getPoseEstimate();
    }

    /**
     * Gets the translation in state using the delta in the tracking wheel pose.
     * @return Translational vector
     */
    @Override
    public  RealVector getTranslationVector() {
        prevState = currentState;
        currentState = PoseUtils.poseToVector(trackingWheelPose);
        RealVector translation = currentState.subtract(prevState);

        return translation;
    }

    @Override
    public Pose2d getRawEstimate() {
        return trackingWheelPose;
    }


}
