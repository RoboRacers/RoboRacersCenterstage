package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.readings.TestSensorStack;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.PoseUtils;
import org.jetbrains.annotations.TestOnly;

@TestOnly
public class TestMotionModel implements MotionModel{

    RealVector prevState;
    RealVector currentState;

    public TestMotionModel(Pose2d startPose) {
        currentState = PoseUtils.poseToVecor(startPose);
    }

    /**
     * Gets the translation in state using the delta in the tracking wheel pose.
     * @return
     */
    @Override
    public  RealVector getTranslationVector() {
        prevState = currentState;
        currentState = PoseUtils.poseToVecor(TestSensorStack.mockPoseEstimate);
        RealVector translation = currentState.subtract(prevState);

        return translation;
    }

    @Override
    public void update() {

    }
}
