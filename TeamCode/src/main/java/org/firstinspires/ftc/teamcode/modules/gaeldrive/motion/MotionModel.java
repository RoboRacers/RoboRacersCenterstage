package org.firstinspires.ftc.teamcode.modules.gaeldrive.motion;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.Updatable;

/**
 * A model to predict the movement of a robot over a given timeframe.
 */
public interface MotionModel extends Updatable {

    public RealVector getTranslationVector();

}
