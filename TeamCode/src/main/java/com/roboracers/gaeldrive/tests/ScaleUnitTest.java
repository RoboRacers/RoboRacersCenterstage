package com.roboracers.gaeldrive.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.roboracers.gaeldrive.filters.ParticleFilter2d;
import com.roboracers.gaeldrive.sensors.SensorModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.TestDistanceSensorModel;
import com.roboracers.gaeldrive.utils.Updatable;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.PoseUtils;

import java.util.ArrayList;
import java.util.List;

public class ScaleUnitTest {
    static long loop;
    static long loopTime = 0;

    static ParticleFilter2d filter = new ParticleFilter2d(new ParticleFilter2d.Bound(-72,72,-72,72,-0.001,0.001));
    static List<SensorModel> models = new ArrayList<>();

    public static void main(String[] args) throws Exception {

        System.out.println("* * * * * * * * * * * *");
        System.out.println("Unit Test Started!");
        loop = System.nanoTime();

        Pose2d pose1 =  new Pose2d(0,0, 0);
        Pose2d pose2 = new Pose2d(0,0,Math.toRadians(-90));
        Pose2d pose3 = new Pose2d(0,0,Math.toRadians(90));

        models.add(new TestDistanceSensorModel(51, pose1));
        models.add(new TestDistanceSensorModel(51, pose2));
        models.add(new TestDistanceSensorModel(51, pose3));

        for (Updatable model: models
             ) {
            model.update();
        }

        System.out.println("Actual Sensor Reading: " + models.get(0).getActualReading() + ", Relative Sensor Location " + pose1);
        System.out.println("Actual Sensor Reading: " + models.get(1).getActualReading() + ", Relative Sensor Location: " + pose2);
        System.out.println("Actual Sensor Reading: " + models.get(2).getActualReading() + ", Relative Sensor Location: " + pose3);

        filter.initializeParticles(200, new ArrayRealVector(new double[] {0, 0,Math.toRadians(45)}));
        filter.weighParticles(models);
        loopTime = System.nanoTime();

        System.out.println("Best Particle Pose: " + filter.getBestParticle().getState());
        System.out.println("Best Particle Weight: " + filter.getBestParticle().getWeight());
        System.out.println("Time for weighting function call: " + (loopTime-loop)/1000000 + "ms");

        // Start Resampling
        for (int i = 0; i < 19; i++) {
            filter.resampleParticles(new double[] {0.5,0.5,0.01});
            filter.weighParticles(models);
        }
        loopTime = System.nanoTime();

        System.out.println("Random Resampled Particle: " + PoseUtils.vectorToPose(filter.getRandomParticle().getState()));
        System.out.println("Time for resampling function call: " + (loopTime-loop)/1000000 + "ms");




        System.out.println("Best Particle after cycles: " + filter.getBestParticle().getState());

        System.out.println("Random Resampled Particle: " + PoseUtils.vectorToPose(filter.getRandomParticle().getState()));

        System.out.println("Unit Test Ended!");
        System.out.println("* * * * * * * * * * * *");
    }
}
