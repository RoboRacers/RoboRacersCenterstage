package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.roboracers.gaeldrive.motion.MotionModel;

import org.apache.commons.math3.linear.RealVector;

import com.roboracers.gaeldrive.utils.MismatchedLengthException;
import com.roboracers.gaeldrive.utils.PoseUtils;
import com.roboracers.gaeldrive.utils.StatsUtils;

/**
 * Motion model that uses a Roadrunner Localizer to generate a translation delta.
 * @see MotionModel
 */
public class RRLocalizerMotionModel implements MotionModel {

    Localizer localizer;

    Pose2d trackingWheelPose;

    RealVector prevState;
    RealVector currentState;

    double[] deviance = {0.05, 0.05, 0.01};


    public RRLocalizerMotionModel(Pose2d startPose, Localizer localizer) {
        currentState = PoseUtils.poseToVector(startPose);
        this.localizer = localizer;
    }

    public void update() {
        localizer.update();
        trackingWheelPose = localizer.getPoseEstimate();
    }

    /**
     * Gets the translation in state using the delta in the tracking wheel pose.
     * @return Translational vector
     */
    @Override
    public  RealVector getTranslationVector() throws Exception {
        prevState = currentState;
        currentState = PoseUtils.poseToVector(trackingWheelPose);
        RealVector translation = currentState.subtract(prevState);

        return StatsUtils.addGaussianNoise(translation, deviance);
    }

    @Override
    public Pose2d getRawEstimate() {
        return trackingWheelPose;
    }

}
