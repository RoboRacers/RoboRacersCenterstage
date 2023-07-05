package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorBuffer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.Pose2dUtils;

public class ThreeWheelMotionModel implements MotionModel{

    RealVector prevState;
    RealVector currentState;

    public ThreeWheelMotionModel(Pose2d startPose) {
        currentState = Pose2dUtils.poseToVecor(startPose);
    }

    /**
     * Gets the translation in state using the delta in the tracking wheel pose.
     * @return
     */
    @Override
    public  RealVector getTranslationVector() {
        prevState = currentState;
        currentState = Pose2dUtils.poseToVecor(SensorBuffer.trackingWheelPose);
        RealVector translation = currentState.subtract(prevState);

        return translation;
    }
}
