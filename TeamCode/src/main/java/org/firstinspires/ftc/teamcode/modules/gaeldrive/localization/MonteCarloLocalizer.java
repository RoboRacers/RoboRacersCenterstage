package org.firstinspires.ftc.teamcode.modules.gaeldrive.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.LocalizationConstants;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.filters.ParticleFilter2d;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.MotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.particles.Particle;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.readings.SensorStack;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.readings.StandardSensorStack;

import java.util.HashMap;
import java.util.List;


/**
 * Localizer class that uses a method called Monte Carlo Localization (MCL) estimate the position of a robot
 * using probabilistic techniques, given a set of sensor values and constraints.
 */
public class MonteCarloLocalizer implements Localizer {

    // Change these in LocalizationConstants only!
    Pose2d  poseEstimate = LocalizationConstants.START_POSE;
    int particleCount = LocalizationConstants.PARTICLE_COUNT;

    ParticleFilter2d particleFilter2d;
    MotionModel motionModel;

    public MonteCarloLocalizer(HardwareMap hardwareMap){

        StandardSensorStack.init(hardwareMap);
        particleFilter2d = new ParticleFilter2d();
        particleFilter2d.initializeParticles(this.particleCount, this.poseEstimate);
        motionModel = new TrackingWheelMotionModel(poseEstimate, hardwareMap);
    }

    @Override
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d;
        particleFilter2d.initializeParticles(particleCount, poseEstimate);
    }


    //TODO: Implement Pose Velocity (Maybe?)
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    public List<Pose2d> getParticlePoses() {
        return particleFilter2d.getParticlePoses();
    }

    public HashMap<Integer, Particle> getParticleMap() {
        return particleFilter2d.Particles;
    }

    /**
     * Runs one localization cycle of Monte Carlo Localization.
     */
    @Override
    public void update() {
        // Update all sensor values
        StandardSensorStack.update();
        // Translate all particles in our particle filter
        particleFilter2d.translateParticles(StandardSensorStack.trackingWheelMotionModel.getTranslationVector());
        // Weigh Particle
        particleFilter2d.weighParticles(StandardSensorStack.getSensorModels());

        poseEstimate = particleFilter2d.getBestPose();

    }

}
