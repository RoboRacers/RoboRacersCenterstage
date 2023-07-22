package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.readings.StandardSensorStack;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.PoseUtils;

/**
 * A motion model that uses the tracking wheel estimate to estimate the movement of a robot in a given timeframe.
 * Used in Monte Carlo Localization
 * @see MotionModel
 */
public class TrackingWheelMotionModel implements MotionModel{

    StandardTrackingWheelLocalizer trackingWheelLocalizer;

    Pose2d trackingWheelPose;

    RealVector prevState;
    RealVector currentState;


    public TrackingWheelMotionModel(Pose2d startPose, HardwareMap hardwareMap) {
        currentState = PoseUtils.poseToVecor(startPose);
        trackingWheelLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
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
        currentState = PoseUtils.poseToVecor(trackingWheelPose);
        RealVector translation = currentState.subtract(prevState);

        return translation;
    }

    public Pose2d getTrackingWheelPose() {
        return trackingWheelPose;
    }


}
