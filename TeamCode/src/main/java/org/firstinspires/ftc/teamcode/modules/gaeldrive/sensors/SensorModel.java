package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.Updatable;

public interface SensorModel extends Updatable {

    public double getWeightModifier();

    public RealVector getActualReading();

    public RealVector getSimulatedReading(RealVector state);



}
