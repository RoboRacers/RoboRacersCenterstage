package com.roboracers.gaeldrive.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.roboracers.gaeldrive.distance.FieldDistance;
import com.roboracers.gaeldrive.filters.ParticleFilter;
import com.roboracers.gaeldrive.filters.ParticleFilter2d;
import com.roboracers.gaeldrive.sensors.SensorModel;

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
        models.add(new TestDistanceSensorModel(51, new Pose2d(0,0,Math.toRadians(-45))));
        models.add(new TestDistanceSensorModel(51, new Pose2d(0,0,Math.toRadians(45))));
        filter.initializeParticles(200000, new Pose2d(0,0,Math.toRadians(45)));
        filter.weighParticles(StandardSensorStack.getSensorModels());
        loopTime = System.nanoTime();
        System.out.println("Time for function call: " + (loopTime-loop) + "Best Pose: " + filter.getBestPose());
    }
}
