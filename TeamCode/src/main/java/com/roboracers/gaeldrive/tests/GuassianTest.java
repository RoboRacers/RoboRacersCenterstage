package com.roboracers.gaeldrive.tests;

import java.util.Random;

public class GuassianTest {

    static Random random = new Random();

    public static void main(String[] args) {
        for (int i = 0; i < 10; i++) {
            System.out.println(random.nextGaussian());
        }
    }
}
