package com.roboracers.gaeldrive.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.roboracers.gaeldrive.distance.FieldDistance;
import com.roboracers.gaeldrive.filters.ParticleFilter2d;

public class MapTest {
    static long loop;
    static long loopTime = 0;
    public static void main(String[] args) {
        loop = System.nanoTime();
        double distance = FieldDistance.calculateSimulatedDistance(new Pose2d(71,71,Math.toRadians(180)));
        loopTime = System.nanoTime();
        System.out.println("Time for function call: " + (loopTime-loop));
        System.out.println(distance);
    }
}
