package org.firstinspires.ftc.teamcode.modules.gaeldrive.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.LocalizationConstants;

import com.roboracers.gaeldrive.filters.ParticleFilter2d;
import com.roboracers.gaeldrive.filters.ParticleFilter2d.Bound;
import com.roboracers.gaeldrive.motion.MotionModel;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.Updatable;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.RRLocalizerMotionModel;
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

    // Change these in LocalizationConstants only!
    Pose2d poseEstimate = LocalizationConstants.START_POSE;
    int particleCount = LocalizationConstants.PARTICLE_COUNT;

    public double[] motionDeviances = {0.005, 0.005, 0.001};

    public double[] resampleDeviances = new double[] {0.1, 0.1, 0.01};

    public Bound initializatioBound = new Bound(-0.5,0.5,-0.5,0.5, -0.001, 0.001);

    public Bound reInitializatioBound = new Bound(-0.5,0.5,-0.5,0.5, -0.001, 0.001);

    ParticleFilter2d particleFilter2d;

    MotionModel motionModel;

    public List<SensorModel> sensorModels = new ArrayList<>();

    public MonteCarloLocalizer(HardwareMap hardwareMap){

        motionModel = new RRLocalizerMotionModel(poseEstimate, new StandardTrackingWheelLocalizer(hardwareMap), motionDeviances);

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

        particleFilter2d = new ParticleFilter2d(initializatioBound, resampleDeviances);
        particleFilter2d.initializeParticles(particleCount, PoseUtils.poseToVector(poseEstimate), initializatioBound);
    }

    @Override
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d;
        particleFilter2d.initializeParticles(particleCount, PoseUtils.poseToVector(poseEstimate));

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
        // Update all sensor values
        motionModel.update();
        for (Updatable sensorModel : sensorModels) {
            sensorModel.update();
        }
        // Translate all particles in our particle filter
        try {
            particleFilter2d.translateParticles(motionModel.getTranslationVector());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Weigh Particles
        try {
            particleFilter2d.weighParticles(sensorModels);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Get the best pose estimate
        poseEstimate = PoseUtils.vectorToPose(particleFilter2d.getBestParticle().getState());

    }

}
