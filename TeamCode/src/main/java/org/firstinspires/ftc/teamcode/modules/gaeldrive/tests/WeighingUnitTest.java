package org.firstinspires.ftc.teamcode.modules.gaeldrive.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.distribution.ChiSquaredDistribution;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.LocalizationConstants;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.filters.ParticleFilter2d;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.localization.MonteCarloLocalizerTest;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.TestSensorModel;

import java.util.ArrayList;
import java.util.List;

public class WeighingUnitTest {

    static ParticleFilter2d testParticleFilter;

    static Pose2d poseEstimate = LocalizationConstants.START_POSE;
    static int particleCount = LocalizationConstants.PARTICLE_COUNT;

    public static void main(String[] args) {
        System.out.println("Unit test starting!");



        testParticleFilter = new ParticleFilter2d();

        testParticleFilter.initializeParticles(particleCount, poseEstimate);

        List<SensorModel> models = new ArrayList<>();
        models.add(new TestSensorModel());

        testParticleFilter.weighParticles(models);

        System.out.println("Best Particle: " + testParticleFilter.getBestParticle().getId());


        System.out.println("Unit test finished!");

    }

}
