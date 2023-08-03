package com.roboracers.gaeldrive.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.roboracers.gaeldrive.distance.FieldDistance;
import com.roboracers.gaeldrive.filters.ParticleFilter;
import com.roboracers.gaeldrive.filters.ParticleFilter2d;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.PoseUtils;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.readings.StandardSensorStack;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.TestDistanceSensorModel;

import java.util.ArrayList;
import java.util.List;

public class WeightingUnitTest {
    static long loop;
    static long loopTime = 0;

    static ParticleFilter2d filter = new ParticleFilter2d();
    static List<SensorModel> models = new ArrayList<>();
    public static void main(String[] args) {
        loop = System.nanoTime();
        models.add(new TestDistanceSensorModel(51, new Pose2d(0,0, 0)));
        models.add(new TestDistanceSensorModel(51, new Pose2d(0,0,Math.toRadians(-90))));
        models.add(new TestDistanceSensorModel(51, new Pose2d(0,0,Math.toRadians(90))));
        for (SensorModel model: models) {
            model.update();
        }
        RealVector simVal1 = models.get(0).getSimulatedReading(PoseUtils.poseToVector(new Pose2d(36,36, Math.toRadians(45))));
        RealVector simVal2 = models.get(2).getSimulatedReading(PoseUtils.poseToVector(new Pose2d(36,36, Math.toRadians(45))));
        RealVector simVal3 = models.get(2).getSimulatedReading(PoseUtils.poseToVector(new Pose2d(36,36, Math.toRadians(45))));
        System.out.println("Actual Sensor Reading: " + models.get(0).getActualReading() + ", Simulated Reading: " + simVal1);
        System.out.println("Actual Sensor Reading: " + models.get(1).getActualReading() + ", Simulated Reading: " + simVal2);
        System.out.println("Actual Sensor Reading: " + models.get(2).getActualReading() + ", Simulated Reading: " + simVal3);
        filter.initializeParticles(200000, new Pose2d(36, 36,Math.toRadians(45)));
        filter.weighParticles(models);
        loopTime = System.nanoTime();
        System.out.println("Time for function call: " + (loopTime-loop) + "Best Pose: " + filter.getBestPose());
    }
}
