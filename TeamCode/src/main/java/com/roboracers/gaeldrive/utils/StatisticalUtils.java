package com.roboracers.gaeldrive.utils;

import com.acmerobotics.roadrunner.util.MathUtil;

import org.apache.commons.math3.linear.RealVector;

import java.util.Random;

public class StatisticalUtils {

    static Random random = new Random();

    static private double STD_DEVIATION = 1;
    static private double MEAN = 0;

    public RealVector addGaussianNoise(RealVector state) {
        int len = state.getDimension();
        for (int i = 0; i < len; i++) {
            state.setEntry(len-1, state.getEntry(len-1) + generateGaussian());
        }
        return state;
    }

    public RealVector addGaussianNoise2D(RealVector state) {
        if (state.getDimension() != 3) {
            return null;
        } else {
            state.setEntry(0, state.getEntry(0) + generateGaussian(1,0));
            state.setEntry(1, state.getEntry(1) + generateGaussian(1,0));
            state.setEntry(2, state.getEntry(2) + generateGaussian(0.01,0));
        }
        return state;
    }


    public static double generateGaussian() {
        return random.nextGaussian() * STD_DEVIATION + MEAN;
    }

    public static double generateGaussian(double STD_DEVIATION, double MEAN) {
        return random.nextGaussian() * STD_DEVIATION + MEAN;
    }

    public static void setStdDeviation(double newSTD_DEVIATION) {
        STD_DEVIATION = newSTD_DEVIATION;
    }

    public static void setMEAN(double newMEAN) {
        MEAN = newMEAN;
    }

}
