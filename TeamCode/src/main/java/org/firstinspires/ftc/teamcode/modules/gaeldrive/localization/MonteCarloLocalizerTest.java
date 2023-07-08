package org.firstinspires.ftc.teamcode.modules.gaeldrive.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.filters.ParticleFilter;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorBuffer;


/**
 * Localizer that uses Monte Carlo Localization (MCL) to get the position of the robot,
 * given a set of sensor values
 */
public class MonteCarloLocalizerTest implements Localizer {

    Pose2d  poseEstimate;
    Pose2d startPose;
    int particleCount = 50;

    ParticleFilter particleFilter;
    TrackingWheelMotionModel motionModel;

    public MonteCarloLocalizerTest(HardwareMap hardwareMap){
        SensorBuffer.init(hardwareMap);
        particleFilter.initializeParticles(particleCount, poseEstimate);
        motionModel = new TrackingWheelMotionModel(poseEstimate);
    }

    /**
     * Getting and Setting the PoseEstimate
     */
    @Override
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d;
        particleFilter.initializeParticles(particleCount, poseEstimate);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    /**
     * Runs one localization cycle
     * This function is inherited from the Localizer class and is used by the drive object.
     */
    @Override
    public void update() {
        SensorBuffer.update();
        particleFilter.translateParticles(motionModel.getTranslationVector());
        poseEstimate = particleFilter.getBestPose();
    }

}
