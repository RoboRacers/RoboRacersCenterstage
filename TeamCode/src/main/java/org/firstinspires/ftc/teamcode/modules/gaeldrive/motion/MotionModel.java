package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import org.apache.commons.math3.linear.RealVector;

/**
 * A model to predict the movement of a robot over a given timeframe.
 */
public interface MotionModel {

    public  RealVector getTranslationVector();

}
