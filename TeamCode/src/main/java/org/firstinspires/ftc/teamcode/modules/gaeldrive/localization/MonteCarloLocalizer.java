package org.firstinspires.ftc.teamcode.modules.gaeldrive.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;

import com.roboracers.gaeldrive.filters.ParticleFilter2d;
import com.roboracers.gaeldrive.filters.ParticleFilter2d.Bound;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.StatsUtils;
import com.roboracers.gaeldrive.utils.Updatable;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.AnalogDistanceSensorModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.PoseUtils;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorUtils;


import java.util.ArrayList;
import java.util.List;


/**
 * Localizer class that uses a method called Monte Carlo Localization (MCL) estimate the position of a robot
 * using probabilistic techniques, given a set of sensor values and constraints.
 */
public class MonteCarloLocalizer implements Localizer {


    /**
     * The particle filter at the heart of monte carlo localization.
     */
    ParticleFilter2d particleFilter2d;
    /**
     * Roadrunner Localizer interface that is used to translate particles periodically.
     * AKA the "Motion Model"
     */
    Localizer internalLocalizer;

    public List<SensorModel> sensorModels = new ArrayList<>();

    Pose2d poseEstimate;

    /*
     * These are the parameters that you can tune to your liking.
     */
    /**
     * This defines the number of particles used in the particle filter.
     * A higher particle leads to more diversity and a higher accuracy and "resolution",
     * But has a high performance cost. Monitor your looptimes when changing this variable to ensure
     * that your performance is not being affected drastically.
     */
    int particleCount = 2000;

    /**
     * This defines the amount of guassian noise applied whenever the particles are
     * translated based on the internal localizer.
     */
    public double[] translationDeviances = {0.005, 0.005, 0.001};
    /**
     * This defines the deviances that are applied when resampling particles as part of
     * the monte carlo cycle.
     */
    public double[] resampleingDeviances = {0.1, 0.1, 0.01};

    public Bound initializatioBound = new Bound(
            -0.5,0.5,-0.5,0.5, -0.001, 0.001
    );

    public Bound reInitializatioBound = new Bound(
            -0.5,0.5,-0.5,0.5, -0.001, 0.001
    );


    public MonteCarloLocalizer(HardwareMap hardwareMap){

        internalLocalizer = new StandardTrackingWheelLocalizer(hardwareMap); // Change the internal localizer here

        internalLocalizer.setPoseEstimate(poseEstimate);
        currentState = PoseUtils.poseToVector(poseEstimate);

        // Config our Ultrasonic Distance Sensor
        AnalogDistanceSensorModel ultrasonicRight = SensorUtils.createMB1240Sensor(
                hardwareMap.get(AnalogInput.class,"ultrasonicRight"),
                1,
                new Pose2d(-7,0, -90)
        );
        sensorModels.add(ultrasonicRight);
        AnalogDistanceSensorModel ultrasonicLeft = SensorUtils.createMB1240Sensor(hardwareMap.get(
                AnalogInput.class,"ultrasonicLeft"),
                1,
                new Pose2d(7,0, 90)
        );
        sensorModels.add(ultrasonicLeft);
        AnalogDistanceSensorModel ultrasonicBack = SensorUtils.createMB1240Sensor(hardwareMap.get(AnalogInput.class,"ultrasonicBack"),
                1,
                new Pose2d(-7,0, 180)
        );
        sensorModels.add(ultrasonicBack);

        particleFilter2d = new ParticleFilter2d(initializatioBound, resampleingDeviances);
        particleFilter2d.initializeParticles(particleCount, PoseUtils.poseToVector(poseEstimate), initializatioBound);
    }

    @Override
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d;
        // Reinitialize particles at the new position
        particleFilter2d.initializeParticles(particleCount, PoseUtils.poseToVector(poseEstimate), reInitializatioBound);
        // Set the state and the internal localizer to the new pose
        prevState = PoseUtils.poseToVector(pose2d);
        currentState = PoseUtils.poseToVector(pose2d);
        internalLocalizer.setPoseEstimate(pose2d);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    /**
     * Runs one localization cycle of Monte Carlo Localization.
     */
    @Override
    public void update() {
        // Update the internal localizer
        internalLocalizerUpdate();

        // Update all sensor values
        for (Updatable sensorModel : sensorModels) {
            sensorModel.update();
        }

        try {
            // Translate all particles in our particle filter
            particleFilter2d.translateParticles(getTranslationVector());
            // Weigh Particles
            particleFilter2d.weighParticles(sensorModels);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Get the best pose estimate
        poseEstimate = PoseUtils.vectorToPose(particleFilter2d.getBestParticle().getState());

    }

    private RealVector prevState;
    private RealVector currentState;

    private void internalLocalizerUpdate () {
        internalLocalizer.update();
        prevState = currentState;
        currentState = PoseUtils.poseToVector(internalLocalizer.getPoseEstimate());
    }

    private RealVector getTranslationVector() throws Exception {
        RealVector translation = currentState.subtract(prevState);
        return StatsUtils.addGaussianNoise(translation, translationDeviances);
    }

}
