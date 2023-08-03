package com.roboracers.gaeldrive.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.roboracers.gaeldrive.filters.ParticleFilter2d;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.PoseUtils;

import org.apache.commons.math3.linear.RealVector;

import com.roboracers.gaeldrive.sensors.TestDistanceSensorModel;

import java.util.ArrayList;
import java.util.List;

public class WeightingUnitTest {
    static long loop;
    static long loopTime = 0;

    static ParticleFilter2d filter = new ParticleFilter2d();
    static List<SensorModel> models = new ArrayList<>();
    public static void main(String[] args) {
        loop = System.nanoTime();
        models.add(new TestDistanceSensorModel(72.001, new Pose2d(0,0, 0)));
        models.add(new TestDistanceSensorModel(72.001, new Pose2d(0,0,Math.toRadians(-90))));
        models.add(new TestDistanceSensorModel(72.001, new Pose2d(0,0,Math.toRadians(90))));
        for (SensorModel model: models) {
            model.update();
        }
        RealVector simVal1 = models.get(0).getSimulatedReading(PoseUtils.poseToVector(new Pose2d(0,0, Math.toRadians(0))));
        RealVector simVal2 = models.get(2).getSimulatedReading(PoseUtils.poseToVector(new Pose2d(0,0, Math.toRadians(0))));
        RealVector simVal3 = models.get(2).getSimulatedReading(PoseUtils.poseToVector(new Pose2d(0,0, Math.toRadians(0))));
        System.out.println("Actual Sensor Reading: " + models.get(0).getActualReading() + ", Simulated Reading: " + simVal1);
        System.out.println("Actual Sensor Reading: " + models.get(1).getActualReading() + ", Simulated Reading: " + simVal2);
        System.out.println("Actual Sensor Reading: " + models.get(2).getActualReading() + ", Simulated Reading: " + simVal3);
        filter.initializeParticles(200000, new Pose2d(0, 0,Math.toRadians(0)));
        filter.weighParticles(models);
        loopTime = System.nanoTime();

        System.out.println("Best Pose: " + filter.getBestPose());
        System.out.println("Time for function call: " + (loopTime-loop));
    }
}
