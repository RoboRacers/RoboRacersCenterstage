package com.roboracers.gaeldrive.tests;

import com.roboracers.gaeldrive.utils.StatsUtils;
import com.roboracers.gaeldrive.utils.VectorUtils;

public class GaussianTest {

    static long loop = 0;
    static long loopTime = 0;

    public static void main(String[] args) {
        for (int i = 0; i < 2000; i++) {
            loop = System.nanoTime();
            StatsUtils.addGaussianNoise2D(VectorUtils.create3DVector(0,0,0), 0.01, 0.001);
            loopTime = System.nanoTime();

        }
        System.out.println("Time for function call: " + (loopTime-loop) + " ms");
    }
}
