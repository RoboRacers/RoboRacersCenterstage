package org.firstinspires.ftc.teamcode.modules.gaeldrive.tests;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.localization.MonteCarloLocalizerTest;

public class TranslationUnitTest {

    static MonteCarloLocalizerTest localizer;

    public static void main(String[] args) {
        localizer = new MonteCarloLocalizerTest();

        for(int i = 0; i<25; i++) {
            localizer.update();
            System.out.println(localizer.getPoseEstimate());
        }

    }

}
