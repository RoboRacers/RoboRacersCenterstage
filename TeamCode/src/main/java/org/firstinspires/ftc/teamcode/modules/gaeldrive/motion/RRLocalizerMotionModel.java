package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.roboracers.gaeldrive.motion.MotionModel;

import org.apache.commons.math3.linear.RealVector;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.PoseUtils;
import com.roboracers.gaeldrive.utils.StatsUtils;

/**
 * Motion model that uses a Roadrunner Localizer to generate a translation delta.
 * @see MotionModel
 */
public class RRLocalizerMotionModel implements MotionModel {

    public Localizer localizer;

    private RealVector prevState;
    private RealVector currentState;

    public double[] deviance = {0.005, 0.005, 0.001};


    public RRLocalizerMotionModel(Pose2d startPose, Localizer localizer) {
        currentState = PoseUtils.poseToVector(startPose);
        this.localizer = localizer;
    }

    public RRLocalizerMotionModel(Pose2d startPose, Localizer localizer, double[] deviance) {
        currentState = PoseUtils.poseToVector(startPose);
        this.localizer = localizer;
        this.deviance = deviance;
    }

    public void update() {
        localizer.update();
        prevState = currentState;
        currentState = PoseUtils.poseToVector(localizer.getPoseEstimate());
    }

    public void setPoseEstimate(Pose2d pose) {
        prevState = PoseUtils.poseToVector(pose);
        currentState = PoseUtils.poseToVector(pose);
        localizer.setPoseEstimate(pose);
    }

    /**
     * Gets the translation in state using the delta in the tracking wheel pose.
     * @return Translational vector
     */
    @Override
    public  RealVector getTranslationVector() throws Exception {
        RealVector translation = currentState.subtract(prevState);
        return StatsUtils.addGaussianNoise(translation, deviance);
    }

    @Override
    public Pose2d getRawEstimate() {
        return localizer.getPoseEstimate();
    }

}
