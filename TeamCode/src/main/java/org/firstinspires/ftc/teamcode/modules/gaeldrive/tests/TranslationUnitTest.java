package org.firstinspires.ftc.teamcode.modules.gaeldrive.tests;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.localization.MonteCarloLocalizerTest;

public class TranslationUnitTest {

    static MonteCarloLocalizerTest localizer;

    public static void main(String[] args) {
        System.out.println("Unit test starting!");
        localizer = new MonteCarloLocalizerTest();

        for(int i = 0; i<17; i++) {
            System.out.println("Update #: " + i);
            localizer.update();
            System.out.println(localizer.getPoseEstimate());
        }
        System.out.println("Unit test finished!");

    }

}
